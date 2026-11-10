#!/usr/bin/env python3
"""Render a top-down view of the FAR-LIO local map (last /map message) with the
estimated trajectory overlaid.

Usage:
  python3 plot_map.py --bag map.bag --map-topic /map --odom-topic /lidar_odometry \
      --gt-topic /ground_truth --out map_topview.png
"""
import argparse
import os

import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2


def read_last_cloud(bag, topic):
    cloud = None
    for _, msg, _ in bag.read_messages(topics=[topic]):
        cloud = msg
    return cloud


def read_poses(bag, topic):
    out = []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        p = msg.pose.pose.position
        out.append([p.x, p.y, p.z])
    return np.asarray(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--map-topic', default='/map')
    ap.add_argument('--odom-topic', default='/lidar_odometry')
    ap.add_argument('--gt-topic', default='/ground_truth')
    ap.add_argument('--out', required=True)
    ap.add_argument('--range', type=float, default=60.0, help='half window around the trajectory [m]')
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    cloud = read_last_cloud(bag, args.map_topic)
    odom = read_poses(bag, args.odom_topic)
    try:
        gt = read_poses(bag, args.gt_topic)
    except Exception:
        gt = np.zeros((0, 3))
    bag.close()
    if cloud is None:
        raise SystemExit(f'no message on {args.map_topic}')

    pts = np.asarray(list(pc2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True)))
    print(f'map points: {len(pts)}  odom poses: {len(odom)}')
    if len(odom) == 0:
        raise SystemExit('no odometry')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    # Centre the window on the trajectory segment
    cx, cy = np.median(odom[:, 0]), np.median(odom[:, 1])
    sel = (np.abs(pts[:, 0] - cx) < args.range) & (np.abs(pts[:, 1] - cy) < args.range)
    pts = pts[sel]

    fig, ax = plt.subplots(figsize=(13, 9))
    sc = ax.scatter(pts[:, 0], pts[:, 1], c=pts[:, 2], s=0.5, cmap='viridis', linewidths=0)
    ax.plot(odom[:, 0], odom[:, 1], 'r-', lw=2.2, label='FAR-LIO trajectory')
    if len(gt):
        ax.plot(gt[:, 0], gt[:, 1], 'k--', lw=1.2, label='ground truth')
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('FAR-LIO local map (cuVoxelMap, CPU voxel hash map) and trajectory')
    ax.legend(loc='upper right')
    cb = fig.colorbar(sc, ax=ax, shrink=0.8)
    cb.set_label('z [m]')
    fig.tight_layout()
    os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
    fig.savefig(args.out, dpi=130)
    print('saved', args.out)


if __name__ == '__main__':
    main()
