#!/usr/bin/env python3
"""Generate a Mermaid topic-flow diagram from ROS2 node source files.

Parses create_publisher / create_subscription calls in *_node/*.cpp
and outputs a Mermaid graph block to stdout.

Hardware connections (UART, I2C) are not visible in the ROS2 topic layer
and are therefore hardcoded at the bottom of this script.

Usage:
    python3 scripts/gen_topic_graph.py
    python3 scripts/gen_topic_graph.py --exclude monitor_node
"""

import argparse
import re
from pathlib import Path


def extract_node_info(filepath: Path) -> dict:
    content = filepath.read_text()

    # Node name is registered as Node("name") in the constructor.
    node_match = re.search(r'\bNode\("([^"]+)"', content)
    node_name = node_match.group(1) if node_match else filepath.parent.name

    # create_publisher<Type>("topic", qos)
    publishers = re.findall(
        r'create_publisher<[^>]+>\(\s*"([^"]+)"', content
    )
    # create_subscription<Type>("topic", qos, callback)
    subscribers = re.findall(
        r'create_subscription<[^>]+>\(\s*"([^"]+)"', content
    )

    return {
        "node": node_name,
        "publishes": publishers,
        "subscribes": subscribers,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--exclude",
        nargs="*",
        default=[],
        metavar="NODE",
        help="ROS2 node names to exclude from the graph",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parent.parent
    node_files = sorted(root.glob("*_node/*.cpp"))

    nodes = [extract_node_info(f) for f in node_files]
    nodes = [n for n in nodes if n["node"] not in args.exclude]
    active_nodes = {n["node"] for n in nodes}

    # Collect publisher and subscriber lists per topic.
    topic_pubs: dict[str, list[str]] = {}
    topic_subs: dict[str, list[str]] = {}
    for info in nodes:
        for topic in info["publishes"]:
            topic_pubs.setdefault(topic, []).append(info["node"])
        for topic in info["subscribes"]:
            topic_subs.setdefault(topic, []).append(info["node"])

    lines = ["graph LR"]

    # Legend subgraph (invisible anchor nodes, visible edges only).
    lines += [
        "    subgraph Legend",
        '        _a[ ] -->|ROS2 topic| _b[ ]',
        '        _c[ ] -.->|hardware| _d[ ]',
        "    end",
    ]

    # Hardware connections (physical layer — not derivable from ROS2 topics).
    # Uses dotted arrows to distinguish from ROS2 topic edges.
    lines.append("    VL53L1X -.->|I2C| tof_node")
    lines.append("    roomba_node -.->|commands UART| Roomba")
    lines.append("    Roomba -.->|sensors UART| roomba_node")

    # Topic edges: solid arrows, one per publisher-subscriber pair.
    for topic in sorted(set(topic_pubs) | set(topic_subs)):
        for pub in topic_pubs.get(topic, []):
            for sub in topic_subs.get(topic, []):
                if sub in active_nodes:
                    lines.append(f'    {pub} -->|"{topic}"| {sub}')

    if args.exclude:
        lines.append(f'    %% excluded: {", ".join(sorted(args.exclude))}')

    print("```mermaid")
    print("\n".join(lines))
    print("```")


if __name__ == "__main__":
    main()
