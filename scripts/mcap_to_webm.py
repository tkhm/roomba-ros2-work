#!/usr/bin/env python3
"""Convert a ROS2 MCAP bag to a webm video with sensor data overlay.

Reads /image_raw/compressed (camera), /roomba/sensors, and /roomba/drive_command
from the bag, overlays sensor values on each frame, and encodes as webm.

Usage:
    python3 scripts/mcap_to_webm.py ~/bags/session1
    python3 scripts/mcap_to_webm.py ~/bags/session1 -o ~/videos/session1.webm
    python3 scripts/mcap_to_webm.py ~/bags/session1 --scale 0.5   # half resolution

Requirements:
    sudo apt install ffmpeg
    ROS2 environment must be sourced (rosbag2_py, rclpy)
    roomba_msgs must be colcon-built and sourced
"""

import argparse
import bisect
import datetime
import subprocess
import sys

import cv2
import numpy as np
import rclpy.serialization
import rosbag2_py
from sensor_msgs.msg import CompressedImage

try:
    from roomba_msgs.msg import DriveCommand, RoombaSensors
except ImportError:
    print('ERROR: roomba_msgs not found.')
    print('  Build and source it first:')
    print('    cd ~/ros2_ws && colcon build --packages-select roomba_msgs')
    print('    source ~/ros2_ws/install/setup.bash')
    sys.exit(1)


def find_nearest_index(timestamps: list, target: int) -> int | None:
    """Return index of the timestamp nearest to target."""
    if not timestamps:
        return None
    idx = bisect.bisect_left(timestamps, target)
    if idx == 0:
        return 0
    if idx == len(timestamps):
        return len(timestamps) - 1
    if timestamps[idx] - target < target - timestamps[idx - 1]:
        return idx
    return idx - 1


def action_label(left: int, right: int) -> str:
    if left == 0 and right == 0:
        return 'stop'
    if left > 0 and right > 0:
        return 'fwd'
    if left < 0 and right < 0:
        return 'back'
    if left < 0 and right > 0:
        return 'spin-L'
    if left > 0 and right < 0:
        return 'spin-R'
    return 'move'


def draw_overlay(
    frame: np.ndarray,
    sensors: RoombaSensors | None,
    drive: DriveCommand | None,
    total_dist_mm: float,
    total_angle_deg: float,
    frame_ts_ns: int,
    first_ts_ns: int,
) -> np.ndarray:
    """Draw a semi-transparent sensor data bar at the top of the frame."""
    # All layout values are defined at reference width REF_W (640px) and scaled
    # proportionally so the overlay looks the same regardless of output resolution.
    REF_W = 640

    # Overlay bar height at reference width. Contains 4 text lines with ~20px spacing.
    REF_BAR_H = 92

    # Y positions (baseline) of the 4 text lines inside the bar at reference width.
    # Line 1 (y=18): bumpers and cliff sensors
    # Line 2 (y=38): battery voltage/current  |  drive command
    # Line 3 (y=58): accumulated odometry
    # Line 4 (y=78): wall clock timestamp  |  elapsed time
    REF_Y1, REF_Y2, REF_Y3, REF_Y4 = 18, 38, 58, 78

    # Font scale at reference width. cv2.FONT_HERSHEY_SIMPLEX at 0.45 fits
    # the longest label (e.g. "Battery: 16000 mV  +0 mA") within 640px.
    REF_FONT_SCALE = 0.45

    # X offsets between elements on line 1 at reference width (pixels).
    # Tuned so labels don't overlap at 640px; they scale down with the frame.
    REF_X_MARGIN = 8        # left margin
    REF_X_BUMP_LABEL = 46   # width of "Bump:" label
    REF_X_BUMP_L = 72       # width of "L[BUMP]" / "L[    ]"
    REF_X_BUMP_R = 84       # width of "R[BUMP]" + gap to "Cliff:"
    REF_X_CLIFF_LABEL = 46  # width of "Cliff:" label
    REF_X_CLIFF_ITEM = 34   # width per cliff indicator (L / FL / FR / R)

    h, w = frame.shape[:2]
    r = w / REF_W  # scale factor relative to reference width
    bar_h = int(REF_BAR_H * r)

    # Semi-transparent black background at the top
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, bar_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = REF_FONT_SCALE * r
    thick = 1
    white = (255, 255, 255)
    red = (80, 80, 255)    # BGR
    yellow = (0, 200, 255)

    y1 = int(REF_Y1 * r)
    y2 = int(REF_Y2 * r)
    y3 = int(REF_Y3 * r)
    y4 = int(REF_Y4 * r)

    if sensors is not None:
        # Line 1: Bumpers and cliff sensors
        bump_l_col = red if sensors.bump_left else white
        bump_r_col = red if sensors.bump_right else white
        bump_l_str = '[BUMP]' if sensors.bump_left else '[    ]'
        bump_r_str = '[BUMP]' if sensors.bump_right else '[    ]'

        x = int(REF_X_MARGIN * r)
        cv2.putText(frame, 'Bump:', (x, y1), font, scale, white, thick)
        x += int(REF_X_BUMP_LABEL * r)
        cv2.putText(frame, f'L{bump_l_str}', (x, y1), font, scale, bump_l_col, thick)
        x += int(REF_X_BUMP_L * r)
        cv2.putText(frame, f'R{bump_r_str}', (x, y1), font, scale, bump_r_col, thick)
        x += int(REF_X_BUMP_R * r)
        cv2.putText(frame, 'Cliff:', (x, y1), font, scale, white, thick)
        x += int(REF_X_CLIFF_LABEL * r)
        for name, val in [
            ('L', sensors.cliff_left),
            ('FL', sensors.cliff_front_left),
            ('FR', sensors.cliff_front_right),
            ('R', sensors.cliff_right),
        ]:
            col = red if val else white
            label = f'[{name}]' if val else f' {name} '
            cv2.putText(frame, label, (x, y1), font, scale, col, thick)
            x += int(REF_X_CLIFF_ITEM * r)

        # Line 2: Battery
        v_mv = sensors.battery_voltage_mv
        batt_col = red if v_mv < 12000 else (yellow if v_mv < 14000 else white)
        cv2.putText(
            frame,
            f'Battery: {v_mv} mV  {sensors.battery_current_ma:+d} mA',
            (int(REF_X_MARGIN * r), y2), font, scale, batt_col, thick,
        )

    # Drive command (right side of line 2)
    if drive is not None:
        label = action_label(drive.left_mm_s, drive.right_mm_s)
        cv2.putText(
            frame,
            f'Drive: L={drive.left_mm_s:+d} R={drive.right_mm_s:+d} [{label}]',
            (int(w // 2), y2), font, scale, white, thick,
        )

    # Line 3: Accumulated odometry
    cv2.putText(
        frame,
        f'Odometry: {total_dist_mm:+.0f} mm  {total_angle_deg:+.1f} deg',
        (int(REF_X_MARGIN * r), y3), font, scale, white, thick,
    )

    # Line 4: Wall clock timestamp and elapsed time since recording start.
    # frame_ts_ns is nanoseconds since Unix epoch (from ROS system clock).
    wall_time = datetime.datetime.fromtimestamp(frame_ts_ns / 1e9)
    elapsed_s = (frame_ts_ns - first_ts_ns) / 1e9
    cv2.putText(
        frame,
        f'{wall_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-5]}  +{elapsed_s:.1f}s',
        (int(REF_X_MARGIN * r), y4), font, scale, white, thick,
    )

    return frame


def get_actual_image_fps(bag_path: str) -> tuple[float, int]:
    """Scan image timestamps to calculate the actual capture frame rate and total frame count.

    This is necessary because v4l2_camera may not honour the time_per_frame
    parameter and can deliver frames at a different rate (e.g. 30fps when 15fps
    was requested). Using the actual fps prevents the output video from running
    at the wrong speed.

    Returns (fps, total_frame_count).
    """
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/image_raw/compressed']))

    first_ts: int | None = None
    last_ts: int | None = None
    count = 0
    while reader.has_next():
        _, _, ts = reader.read_next()
        if first_ts is None:
            first_ts = ts
        last_ts = ts
        count += 1

    if count < 2 or first_ts is None or last_ts is None:
        return 15.0, count  # fallback if bag is too short to measure

    duration_s = (last_ts - first_ts) / 1e9
    return count / duration_s, count


def read_sensor_data(
    bag_path: str,
) -> tuple[list, list, list, list, list]:
    """Pass 1: read all sensor and drive command messages."""
    sensor_ts: list[int] = []
    sensor_msgs: list[RoombaSensors] = []
    sensor_odometry: list[tuple[float, float]] = []
    drive_ts: list[int] = []
    drive_msgs: list[DriveCommand] = []

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    reader.set_filter(rosbag2_py.StorageFilter(
        topics=['/roomba/sensors', '/roomba/drive_command'],
    ))

    total_dist = 0.0
    total_angle = 0.0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic == '/roomba/sensors':
            msg = rclpy.serialization.deserialize_message(data, RoombaSensors)
            total_dist += msg.distance_mm
            total_angle += msg.angle_degrees
            sensor_ts.append(ts)
            sensor_msgs.append(msg)
            sensor_odometry.append((total_dist, total_angle))
        elif topic == '/roomba/drive_command':
            msg = rclpy.serialization.deserialize_message(data, DriveCommand)
            drive_ts.append(ts)
            drive_msgs.append(msg)

    return sensor_ts, sensor_msgs, sensor_odometry, drive_ts, drive_msgs


def encode_video(
    bag_path: str,
    output: str,
    actual_fps: float,
    speed: float,
    scale: float,
    total_frames: int,
    sensor_ts: list,
    sensor_msgs: list,
    sensor_odometry: list,
    drive_ts: list,
    drive_msgs: list,
) -> int:
    """Pass 2: decode image frames, draw overlay, encode to webm.

    output_fps = actual_fps * speed
      actual_fps: measured from bag timestamps (avoids duration mismatch when
                  the camera delivers frames faster than the requested rate)
      speed:      playback speed multiplier (e.g. 3.0 = 3x faster than real time)
    """
    output_fps = actual_fps * speed
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/image_raw/compressed']))

    ffmpeg: subprocess.Popen | None = None
    first_ts: int | None = None
    count = 0

    while reader.has_next():
        _, data, ts = reader.read_next()
        if first_ts is None:
            first_ts = ts
        msg = rclpy.serialization.deserialize_message(data, CompressedImage)

        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if frame is None:
            continue

        # Resize if scale < 1.0
        if scale != 1.0:
            new_w = int(frame.shape[1] * scale)
            new_h = int(frame.shape[0] * scale)
            frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)

        # Start ffmpeg on the first frame (need frame dimensions)
        if ffmpeg is None:
            h, w = frame.shape[:2]
            ffmpeg = subprocess.Popen(
                [
                    'ffmpeg', '-y',
                    '-f', 'rawvideo',
                    '-pix_fmt', 'bgr24',
                    '-framerate', str(output_fps),
                    '-video_size', f'{w}x{h}',
                    '-i', 'pipe:0',
                    '-c:v', 'libvpx-vp9',
                    '-b:v', '1M',
                    output,
                ],
                stdin=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )

        s_idx = find_nearest_index(sensor_ts, ts)
        d_idx = find_nearest_index(drive_ts, ts)
        sensors = sensor_msgs[s_idx] if s_idx is not None else None
        drive = drive_msgs[d_idx] if d_idx is not None else None
        odo = sensor_odometry[s_idx] if s_idx is not None else (0.0, 0.0)

        frame = draw_overlay(frame, sensors, drive, odo[0], odo[1], ts, first_ts)
        ffmpeg.stdin.write(frame.tobytes())

        count += 1
        if count % 30 == 0:
            pct = count / total_frames * 100 if total_frames > 0 else 0
            print(f'\r  {count} / {total_frames} frames ({pct:.0f}%)', end='', flush=True)

    if ffmpeg is None:
        print('No image frames found in bag.')
        return 1

    ffmpeg.stdin.close()
    return ffmpeg.wait()


def main() -> None:
    parser = argparse.ArgumentParser(
        description='Convert MCAP bag to webm video with sensor data overlay',
    )
    parser.add_argument('bag_path', help='Path to the ros2 bag directory')
    parser.add_argument(
        '-o', '--output',
        help='Output .webm file path (default: <bag_path>.webm)',
    )
    parser.add_argument(
        '--speed', type=float, default=2.0,
        help='Playback speed multiplier, e.g. 2.0 = 2x faster than real time (default: 2.0)',
    )
    parser.add_argument(
        '--scale', type=float, default=0.7,
        help='Resize scale factor 0.1–1.0, e.g. 0.5 for half resolution (default: 0.7)',
    )
    args = parser.parse_args()

    bag_path = args.bag_path.rstrip('/')
    output = args.output or bag_path + '.webm'

    print('Pass 1: measuring actual frame rate...')
    actual_fps, total_frames = get_actual_image_fps(bag_path)
    output_fps = actual_fps * args.speed
    print(f'  total frames: {total_frames}  actual fps: {actual_fps:.1f}  speed: {args.speed}x  output fps: {output_fps:.1f}')

    print('Pass 2: reading sensor data...')
    sensor_ts, sensor_msgs, sensor_odometry, drive_ts, drive_msgs = read_sensor_data(bag_path)
    print(f'  sensor msgs: {len(sensor_msgs)},  drive msgs: {len(drive_msgs)}')

    print('Pass 3: encoding video...')
    ret = encode_video(
        bag_path, output, actual_fps, args.speed, args.scale, total_frames,
        sensor_ts, sensor_msgs, sensor_odometry,
        drive_ts, drive_msgs,
    )

    if ret == 0:
        print(f'\nDone: {output}')
    else:
        print(f'\nffmpeg exited with code {ret}')
    sys.exit(ret)


if __name__ == '__main__':
    main()
