#!/usr/bin/env python3
"""Print TYPE and count from an ArduPilot .bin/.log file.

Example:
    python Tools/bin_type_counter.py /path/to/log.bin
    python Tools/bin_type_counter.py /path/to/log.bin | head
"""

import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
MAVLINK_DIR = os.path.join(REPO_ROOT, "modules", "mavlink")
if os.path.isdir(MAVLINK_DIR):
    sys.path.insert(0, MAVLINK_DIR)

try:
    from pymavlink import mavutil
except Exception as exc:  # pragma: no cover
    print(f"ERROR: failed to import pymavlink: {exc}", file=sys.stderr)
    print("Install pymavlink or run this from the ardupilot repository root.", file=sys.stderr)
    sys.exit(1)


def count_types(log_path: str):
    """Return a dict of type name -> count."""
    conn = mavutil.mavlink_connection(log_path)
    counts = {}

    while True:
        msg = conn.recv_match(blocking=False)
        if msg is None:
            break
        msg_type = msg.get_type()
        counts[msg_type] = counts.get(msg_type, 0) + 1

    return counts


def main():
    if len(sys.argv) != 2:
        print(f"Usage: {os.path.basename(sys.argv[0])} <log.bin|log.log>")
        sys.exit(1)

    log_path = sys.argv[1]
    if not os.path.exists(log_path):
        print(f"ERROR: file not found: {log_path}", file=sys.stderr)
        sys.exit(1)

    counts = count_types(log_path)

    print("TYPE,count")
    for msg_type, count in sorted(counts.items(), key=lambda kv: (-kv[1], kv[0])):
        print(f"{msg_type},{count}")


if __name__ == "__main__":
    main()
