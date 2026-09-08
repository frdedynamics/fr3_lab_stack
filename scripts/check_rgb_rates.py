#!/usr/bin/env python3
# Copyright 2026 frdedynamics
# SPDX-License-Identifier: Apache-2.0
"""Bounded RGB reception check; retain counters only, never image data."""

import argparse
import time

import rclpy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=15.0)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')
    rclpy.init(args=[])
    node = rclpy.create_node('fr3_rgb_rate_check')
    counts = {name: {'count': 0, 'max_gap': 0.0}
              for name in ('wrist_camera', 'external_camera')}

    def receive(data, name):
        now = time.monotonic()
        stats = counts[name]
        if stats['count'] == 0:
            stats['first'] = now
            msg = deserialize_message(data, Image)
            stats['profile'] = f'{msg.width}x{msg.height} {msg.encoding}'
        else:
            stats['max_gap'] = max(stats['max_gap'], now - stats['last'])
        stats['last'] = now
        stats['count'] += 1

    for name in counts:
        # Default ROS subscription QoS (reliable, volatile, depth 10).
        # Raw reception avoids repeatedly deserializing multi-megabyte images.
        node.create_subscription(
            Image, f'/camera/{name}/color/image_raw',
            lambda data, name=name: receive(data, name), 10, raw=True,
        )
    try:
        end = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        for name, stats in counts.items():
            if stats['count'] < 2:
                print(f'{name}: insufficient samples ({stats["count"]})')
                continue
            rate = (stats['count'] - 1) / (stats['last'] - stats['first'])
            print(f'{name}: {rate:.3f} Hz, {stats["count"]} images, '
                  f'{stats["profile"]}, max arrival gap {stats["max_gap"]:.3f} s')
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if all(s['count'] >= 2 for s in counts.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
