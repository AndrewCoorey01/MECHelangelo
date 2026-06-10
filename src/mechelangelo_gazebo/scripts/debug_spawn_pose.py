#!/usr/bin/env python3
"""Spawn a Gazebo model and poll its resolved pose.

Run this while gzserver is already running. It is intentionally small: the
pose timeline quickly shows whether a model is inserted at the requested pose
and then moved by physics.
"""

import argparse
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory


def run(command):
    return subprocess.run(command, check=False, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def default_sdf_path():
    gazebo_pkg = get_package_share_directory('mechelangelo_gazebo')
    return f'{gazebo_pkg}/models/mechelangelo_final/model.sdf'


def main():
    parser = argparse.ArgumentParser(
        description='Spawn an SDF into Gazebo and print pose samples.')
    parser.add_argument('--sdf', default=default_sdf_path(),
                        help='SDF file to spawn.')
    parser.add_argument('--name', default='mechelangelo_debug',
                        help='Gazebo entity name.')
    parser.add_argument('--x', default='0')
    parser.add_argument('--y', default='0')
    parser.add_argument('--z', default='0.5')
    parser.add_argument('--roll', default='0')
    parser.add_argument('--pitch', default='0')
    parser.add_argument('--yaw', default='0')
    parser.add_argument('--samples', type=int, default=8)
    parser.add_argument('--interval', type=float, default=0.5)
    args = parser.parse_args()

    spawn = [
        'gz', 'model',
        '-m', args.name,
        '-f', args.sdf,
        '-x', args.x,
        '-y', args.y,
        '-z', args.z,
        '-R', args.roll,
        '-P', args.pitch,
        '-Y', args.yaw,
    ]
    result = run(spawn)
    if result.returncode != 0:
        sys.stderr.write(result.stderr)
        return result.returncode

    print(f'spawned {args.name} from {args.sdf}')
    for index in range(args.samples):
        pose = run(['gz', 'model', '-m', args.name, '-p'])
        stamp = index * args.interval
        if pose.returncode == 0:
            print(f'{stamp:5.2f}s  {pose.stdout.strip()}')
        else:
            print(f'{stamp:5.2f}s  pose query failed: {pose.stderr.strip()}')
        time.sleep(args.interval)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
