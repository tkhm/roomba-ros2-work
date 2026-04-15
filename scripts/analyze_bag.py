#!/usr/bin/env python3
"""Analyze a Roomba ROS2 bag file and compute wall-following performance metrics.

Reads /tof/distance_mm, /roomba/drive_command, /planner/state, and
/roomba/sensors from an MCAP bag and outputs a JSON metrics report.

Usage:
  python3 scripts/analyze_bag.py <bag_dir_or_file.mcap> [options]

Options:
  --out PATH             Output JSON path (default: print to stdout)
  --target-distance INT  Target wall distance in mm (default: 130)
  --label TEXT           Descriptive label stored in the JSON (e.g. "baseline")

Example:
  python3 scripts/analyze_bag.py ~/bags/session_20260414_120000 \\
      --out docs/metrics/2026-04-14_baseline.json \\
      --target-distance 130 \\
      --label "baseline before task1"

Output JSON keys:
  label                 : descriptive label
  bag_path              : input path
  duration_s            : total bag duration in seconds
  target_distance_mm    : wall target (from --target-distance)
  wall_error_rmse_mm    : RMSE of (actual_distance - target_distance) [mm]
  wall_error_max_mm     : maximum absolute distance error [mm]
  avg_speed_mm_s        : mean of (left + right) / 2
  speed_variance        : variance of per-tick speed (ギクシャク度の指標)
  recovery_count        : number of transitions into RECOVERING state
  following_ratio       : fraction of time in FOLLOWING state (0.0–1.0)
  state_durations_s     : {"SEARCHING": s, "FOLLOWING": s, "RECOVERING": s}
  sample_count          : number of /tof/distance_mm messages processed
  notes                 : list of warning strings (missing topics etc.)
"""

import argparse
import json
import math
import os
import sys
from pathlib import Path


def _open_reader(bag_path: str):
    """Return a rosbag2_py SequentialReader for the given path."""
    try:
        import rosbag2_py  # noqa: PLC0415
    except ImportError:
        print("ERROR: rosbag2_py not found. Source your ROS2 workspace first:", file=sys.stderr)
        print("  source /opt/ros/jazzy/setup.bash", file=sys.stderr)
        sys.exit(1)

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def _get_type_map(reader) -> dict:
    """Return {topic_name: type_string} for all topics in the bag."""
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def _deserialize(data: bytes, type_str: str):
    """Deserialize a CDR-encoded message given its ROS2 type string."""
    from rclpy.serialization import deserialize_message  # noqa: PLC0415
    from rosidl_runtime_py.utilities import get_message  # noqa: PLC0415

    msg_type = get_message(type_str)
    return deserialize_message(data, msg_type)


def analyze(bag_path: str, target_distance_mm: int, label: str) -> dict:
    """Read the bag and compute performance metrics."""
    reader = _open_reader(bag_path)
    type_map = _get_type_map(reader)

    topic_tof = "/tof/distance_mm"
    topic_cmd = "/roomba/drive_command"
    topic_state = "/planner/state"
    topic_sensors = "/roomba/sensors"

    available = set(type_map.keys())
    notes = []
    for t in (topic_tof, topic_cmd, topic_state, topic_sensors):
        if t not in available:
            notes.append(f"topic not found in bag: {t}")

    # Accumulators
    tof_samples: list[int] = []
    speeds: list[float] = []  # (left + right) / 2 per tick

    state_seq: list[tuple[int, str]] = []  # (timestamp_ns, state_str)
    prev_state: str = ""
    recovery_count: int = 0

    first_ts: int | None = None
    last_ts: int | None = None

    while reader.has_next():
        topic, data, ts_ns = reader.read_next()

        if first_ts is None:
            first_ts = ts_ns
        last_ts = ts_ns

        if topic == topic_tof and topic_tof in available:
            msg = _deserialize(data, type_map[topic])
            tof_samples.append(int(msg.data))

        elif topic == topic_cmd and topic_cmd in available:
            msg = _deserialize(data, type_map[topic])
            avg = (int(msg.left_mm_s) + int(msg.right_mm_s)) / 2.0
            speeds.append(avg)

        elif topic == topic_state and topic_state in available:
            msg = _deserialize(data, type_map[topic])
            state_str = str(msg.data)
            state_seq.append((ts_ns, state_str))
            if state_str == "RECOVERING" and prev_state != "RECOVERING":
                recovery_count += 1
            prev_state = state_str

    # Total duration
    duration_s: float = 0.0
    if first_ts is not None and last_ts is not None:
        duration_s = (last_ts - first_ts) / 1e9

    # --- Wall distance error metrics ---
    wall_error_rmse_mm: float = 0.0
    wall_error_max_mm: float = 0.0
    if tof_samples:
        errors = [abs(d - target_distance_mm) for d in tof_samples if d > 0]
        if errors:
            wall_error_rmse_mm = math.sqrt(sum(e * e for e in errors) / len(errors))
            wall_error_max_mm = max(errors)

    # --- Speed metrics ---
    avg_speed_mm_s: float = 0.0
    speed_variance: float = 0.0
    if speeds:
        avg_speed_mm_s = sum(speeds) / len(speeds)
        speed_variance = sum((s - avg_speed_mm_s) ** 2 for s in speeds) / len(speeds)

    # --- State duration metrics ---
    state_durations: dict[str, float] = {"SEARCHING": 0.0, "FOLLOWING": 0.0, "RECOVERING": 0.0}
    if len(state_seq) >= 2:
        for i in range(len(state_seq) - 1):
            ts_a, s_a = state_seq[i]
            ts_b, _ = state_seq[i + 1]
            dt = (ts_b - ts_a) / 1e9
            if s_a in state_durations:
                state_durations[s_a] += dt
        # Last segment
        if state_seq:
            last_state = state_seq[-1][1]
            last_dt = (last_ts - state_seq[-1][0]) / 1e9 if last_ts else 0.0
            if last_state in state_durations:
                state_durations[last_state] += last_dt

    total_tracked = sum(state_durations.values())
    following_ratio: float = 0.0
    if total_tracked > 0.0:
        following_ratio = state_durations["FOLLOWING"] / total_tracked
    elif topic_state not in available:
        notes.append("following_ratio is 0 because /planner/state was not recorded")

    return {
        "label": label,
        "bag_path": os.path.abspath(bag_path),
        "duration_s": round(duration_s, 2),
        "target_distance_mm": target_distance_mm,
        "wall_error_rmse_mm": round(wall_error_rmse_mm, 1),
        "wall_error_max_mm": round(wall_error_max_mm, 1),
        "avg_speed_mm_s": round(avg_speed_mm_s, 1),
        "speed_variance": round(speed_variance, 1),
        "recovery_count": recovery_count,
        "following_ratio": round(following_ratio, 3),
        "state_durations_s": {k: round(v, 2) for k, v in state_durations.items()},
        "sample_count": len(tof_samples),
        "notes": notes,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute wall-following performance metrics from a ROS2 bag file."
    )
    parser.add_argument("bag", help="Path to the MCAP bag directory or .mcap file")
    parser.add_argument("--out", metavar="PATH", help="Write JSON to this file (default: stdout)")
    parser.add_argument(
        "--target-distance",
        type=int,
        default=130,
        metavar="MM",
        help="Target wall distance in mm (default: 130)",
    )
    parser.add_argument(
        "--label",
        default="",
        metavar="TEXT",
        help="Descriptive label stored in the output JSON",
    )
    args = parser.parse_args()

    bag_path = args.bag
    if not Path(bag_path).exists():
        print(f"ERROR: bag path does not exist: {bag_path}", file=sys.stderr)
        sys.exit(1)

    metrics = analyze(bag_path, args.target_distance, args.label)
    output = json.dumps(metrics, indent=2, ensure_ascii=False)

    if args.out:
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(output, encoding="utf-8")
        print(f"Metrics written to: {out_path}")
        # Print summary to terminal
        print(f"  duration        : {metrics['duration_s']} s")
        print(f"  wall_error_rmse : {metrics['wall_error_rmse_mm']} mm")
        print(f"  wall_error_max  : {metrics['wall_error_max_mm']} mm")
        print(f"  avg_speed       : {metrics['avg_speed_mm_s']} mm/s")
        print(f"  speed_variance  : {metrics['speed_variance']}")
        print(f"  recovery_count  : {metrics['recovery_count']}")
        print(f"  following_ratio : {metrics['following_ratio']:.1%}")
        if metrics["notes"]:
            for note in metrics["notes"]:
                print(f"  NOTE: {note}")
    else:
        print(output)


if __name__ == "__main__":
    main()
