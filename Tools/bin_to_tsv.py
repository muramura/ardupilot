#!/usr/bin/env python3
"""Extract specified messages or fields from an ArduPilot .bin/.log file and output as TSV to stdout.

Usage:
    # 1. Output all fields of a specific message type (e.g. MSG):
    python3 Tools/bin_to_tsv.py log.bin MSG

    # 2. Output specific fields of a message type:
    python3 Tools/bin_to_tsv.py log.bin MSG -f TimeUS,Message

    # 3. Output multiple message types:
    python3 Tools/bin_to_tsv.py log.bin MSG MODE ERR

    # 4. Filter parameters (e.g. only specific parameter names):
    python3 Tools/bin_to_tsv.py log.bin PARM -f Name,Value

    # 5. Pipe or redirect output:
    python3 Tools/bin_to_tsv.py log.bin MSG > msg_log.tsv
"""

import argparse
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
MAVLINK_DIR = os.path.join(REPO_ROOT, "modules", "mavlink")
if os.path.isdir(MAVLINK_DIR):
    sys.path.insert(0, MAVLINK_DIR)

try:
    from pymavlink import mavutil
except Exception as exc:
    print(f"ERROR: failed to import pymavlink: {exc}", file=sys.stderr)
    print("Install pymavlink or run this from the ardupilot repository root.", file=sys.stderr)
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Extract specified items from an ArduPilot DataFlash .bin log as TSV.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("log_path", help="Path to .bin or .log file")
    parser.add_argument(
        "types",
        nargs="*",
        help="Message type(s) to extract (e.g., MSG, MODE, PARM, ERR, EV). If omitted, use --types or all.",
    )
    parser.add_argument(
        "-t",
        "--type",
        dest="types_opt",
        help="Comma-separated message types to extract (e.g. 'MSG,MODE')",
    )
    parser.add_argument(
        "-f",
        "--fields",
        help="Comma-separated field names to extract (e.g. 'TimeUS,Message' or 'Name,Value').",
    )
    parser.add_argument(
        "-d",
        "--delimiter",
        default="\t",
        help="Column delimiter (default: '\\t' for TSV).",
    )
    parser.add_argument(
        "--no-header",
        action="store_true",
        help="Do not print the header line.",
    )
    return parser.parse_args()


def get_field_value(msg, field_name: str):
    """Safely retrieve and format a field value from a MAVLink/DataFlash message."""
    try:
        val = getattr(msg, field_name)
        if isinstance(val, bytes):
            # Decode bytes if possible
            return val.decode("utf-8", errors="replace").strip("\x00")
        if isinstance(val, str):
            return val.strip("\x00")
        return val
    except AttributeError:
        return ""


def main():
    args = parse_args()

    if not os.path.exists(args.log_path):
        print(f"ERROR: file not found: {args.log_path}", file=sys.stderr)
        sys.exit(1)

    # Collect target message types
    target_types = []
    if args.types:
        for t in args.types:
            for item in t.split(","):
                if item.strip():
                    target_types.append(item.strip().upper())
    if args.types_opt:
        for item in args.types_opt.split(","):
            if item.strip():
                target_types.append(item.strip().upper())

    # Target fields (optional)
    target_fields = []
    if args.fields:
        target_fields = [f.strip() for f in args.fields.split(",") if f.strip()]

    # Connect to log
    conn = mavutil.mavlink_connection(args.log_path)

    # Single message type mode with automatic header detection
    single_type = target_types[0] if len(target_types) == 1 else None
    header_printed = False
    active_fields = target_fields

    # If multiple types, we prefix with 'TYPE' column
    show_type_column = len(target_types) > 1 or len(target_types) == 0

    delim = args.delimiter

    while True:
        msg = conn.recv_match(type=target_types if target_types else None, blocking=False)
        if msg is None:
            break

        msg_type = msg.get_type()

        # Determine fields to print
        if not active_fields:
            if hasattr(msg, "_fieldnames") and msg._fieldnames:
                msg_fields = list(msg._fieldnames)
            elif hasattr(msg, "get_fieldnames") and callable(msg.get_fieldnames):
                msg_fields = list(msg.get_fieldnames())
            else:
                msg_fields = [k for k in dir(msg) if not k.startswith("_") and not callable(getattr(msg, k))]

            if single_type:
                active_fields = msg_fields

        current_fields = active_fields if active_fields else getattr(msg, "_fieldnames", [])

        # Print header if first matching message
        if not header_printed and not args.no_header:
            if show_type_column:
                header_cols = ["TYPE"] + current_fields
            else:
                header_cols = current_fields
            print(delim.join(header_cols))
            header_printed = True

        # Extract values
        row_values = []
        if show_type_column:
            row_values.append(msg_type)

        for field in current_fields:
            val = get_field_value(msg, field)
            row_values.append(str(val))

        print(delim.join(row_values))


if __name__ == "__main__":
    main()
