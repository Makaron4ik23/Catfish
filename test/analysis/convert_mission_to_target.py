#!/usr/bin/env python3
"""
Convert a mission_XX.json (Gazebo world coordinates) into
analysis target waypoints JSON format.

Usage:
    python3 test/analysis/convert_mission_to_target.py path/to/mission_01.json
    python3 test/analysis/convert_mission_to_target.py path/to/mission_01.json -o my_targets.json
"""

import argparse
import json
import sys
from pathlib import Path


def convert(in_path: Path):
    with open(in_path, 'r') as f:
        data = json.load(f)

    waypoints = data.get('waypoints', [])
    if not waypoints:
        print('Error: no waypoints in mission file', file=sys.stderr)
        sys.exit(1)

    enu_wpts = []
    for i, wp in enumerate(waypoints):
        gz = wp['gazebo_world']
        e = gz['x']
        n = gz['y']
        d = -gz['z']
        enu_wpts.append({
            'n': round(n, 4),
            'e': round(e, 4),
            'd': round(d, 4),
            'label': f'WP{i + 1}',
        })

    result = {
        'description': (
            f'Converted from {in_path.name}. '
            f'Gazebo world ENU coordinates extracted from mission waypoints.'
        ),
        'threshold_m': 0.15,
        'waypoints': enu_wpts,
    }

    return result


def main():
    parser = argparse.ArgumentParser(
        description='Convert mission_XX.json to target waypoints JSON format'
    )
    parser.add_argument('mission_file', type=Path, help='Path to mission_XX.json')
    parser.add_argument('-o', '--output', type=Path, default=None,
                        help='Output path (default: stdout)')
    parser.add_argument('--threshold', type=float, default=0.15,
                        help='Distance threshold in metres (default: 0.15)')
    args = parser.parse_args()

    if not args.mission_file.exists():
        print(f'Error: {args.mission_file} not found', file=sys.stderr)
        sys.exit(1)

    result = convert(args.mission_file)
    result['threshold_m'] = args.threshold

    out = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(out)
        print(f'Wrote {args.output}')
    else:
        sys.stdout.write(out)


if __name__ == '__main__':
    main()
