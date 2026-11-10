#!/usr/bin/env python3
"""Evaluate a FAR-LIO odometry run recorded in a ROS 1 bag.

Reads /lidar_odometry and /ground_truth (nav_msgs/Odometry), associates the poses
by timestamp, aligns the estimate to the ground truth with a rigid SE(3)
transform (Umeyama, no scale) and reports ATE / RPE statistics + a plot.

Usage:
  python3 evaluate_trajectory.py --bag full.bag --out /tmp/farlio_full/eval
"""
import argparse
import os

import numpy as np
import rosbag


def poses_from_topic(bag, topic):
    stamps, xyz, quat = [], [], []
    for _, msg, _ in bag.read_messages(topics=[topic]):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        stamps.append(msg.header.stamp.to_sec())
        xyz.append([p.x, p.y, p.z])
        quat.append([q.x, q.y, q.z, q.w])
    order = np.argsort(stamps)
    return np.asarray(stamps)[order], np.asarray(xyz)[order], np.asarray(quat)[order]


def quat_to_rot(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def interpolate(stamps_src, values, stamps_query):
    out = np.zeros((len(stamps_query), values.shape[1]))
    for d in range(values.shape[1]):
        out[:, d] = np.interp(stamps_query, stamps_src, values[:, d])
    return out


def umeyama(src, dst):
    """Rigid transform (R, t) minimising ||dst - (R @ src + t)||."""
    mu_s = src.mean(axis=0)
    mu_d = dst.mean(axis=0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    u, _, vt = np.linalg.svd(cov)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[2, 2] = -1
    r = u @ s @ vt
    return r, mu_d - r @ mu_s


def rpe(est, gt, lengths):
    """Relative translation error for pose pairs separated by >= length (m)."""
    out = {}
    for length in lengths:
        errs = []
        for i in range(len(gt)):
            d = np.linalg.norm(gt[i + 1:, :2] - gt[i, :2], axis=1)
            if len(d) == 0 or d.max() < length:
                continue
            j = i + 1 + int(np.argmax(d >= length))
            errs.append(np.linalg.norm((est[j] - est[i]) - (gt[j] - gt[i])))
        out[length] = np.asarray(errs)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', required=True)
    ap.add_argument('--out', default=None)
    ap.add_argument('--gt-topic', default='/ground_truth')
    ap.add_argument('--est-topic', default='/lidar_odometry')
    args = ap.parse_args()

    bag = rosbag.Bag(args.bag)
    ts_est, p_est, _ = poses_from_topic(bag, args.est_topic)
    ts_gt, p_gt, _ = poses_from_topic(bag, args.gt_topic)
    bag.close()
    print(f'estimates: {len(ts_est)}   ground truth: {len(ts_gt)}')

    common = (ts_est >= ts_gt[0]) & (ts_est <= ts_gt[-1])
    ts = ts_est[common]
    pe = p_est[common]
    pg = interpolate(ts_gt, p_gt, ts)

    r, t = umeyama(pe, pg)
    pa = (r @ pe.T).T + t
    err = np.linalg.norm(pa - pg, axis=1)

    path_len = np.sum(np.linalg.norm(np.diff(pg[:, :2], axis=0), axis=1))
    print(f'path length          : {path_len:.1f} m')
    print(f'ATE RMSE             : {np.sqrt(np.mean(err ** 2)):.3f} m')
    print(f'ATE mean / max       : {np.mean(err):.3f} / {np.max(err):.3f} m')
    print(f'final position error : {err[-1]:.3f} m  ({100 * err[-1] / path_len:.2f} % of path)')
    for length, vals in rpe(pa, pg, [1, 10, 100]).items():
        if len(vals):
            print(f'RPE {length:>3d} m         : mean {np.mean(vals):.3f} m '
                  f'({100 * np.mean(vals) / length:.2f} %)  rmse {np.sqrt(np.mean(vals ** 2)):.3f} m')
    print(f'RPE total            : mean {np.mean(np.linalg.norm(np.diff(pa, axis=0) - np.diff(pg, axis=0), axis=1)):.4f} m/frame')

    if args.out:
        os.makedirs(os.path.dirname(args.out) or '.', exist_ok=True)
        np.savetxt(args.out + '_trajectory.txt',
                   np.column_stack([ts, pa, pg, err]),
                   header='stamp est_x est_y est_z gt_x gt_y gt_z err_m')
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, axes = plt.subplots(2, 2, figsize=(14, 10))
            ax = axes[0, 0]
            ax.plot(pg[:, 0], pg[:, 1], 'k-', lw=2, label='ground truth')
            ax.plot(pa[:, 0], pa[:, 1], 'r--', lw=1.5, label='FAR-LIO')
            ax.set_aspect('equal')
            ax.set_xlabel('x [m]')
            ax.set_ylabel('y [m]')
            ax.set_title('Top view')
            ax.legend()
            ax.grid(alpha=0.3)

            ax = axes[0, 1]
            ax.plot(ts - ts[0], err)
            ax.set_xlabel('time [s]')
            ax.set_ylabel('position error [m]')
            ax.set_title(f'Position error (ATE RMSE {np.sqrt(np.mean(err**2)):.2f} m)')
            ax.grid(alpha=0.3)

            ax = axes[1, 0]
            d = np.cumsum(np.linalg.norm(np.diff(pg, axis=0), axis=1))
            ax.plot(np.concatenate([[0], d]), err, '.', ms=2)
            ax.set_xlabel('travelled distance [m]')
            ax.set_ylabel('error [m]')
            ax.set_title('Error over travelled distance')
            ax.grid(alpha=0.3)

            ax = axes[1, 1]
            ax.plot(np.concatenate([[0], d]), pg[:, 2], 'k-', lw=2, label='ground truth')
            ax.plot(np.concatenate([[0], d]), pa[:, 2], 'r--', lw=1.5, label='FAR-LIO')
            ax.set_xlabel('travelled distance [m]')
            ax.set_ylabel('z [m]')
            ax.set_title('Height profile')
            ax.legend()
            ax.grid(alpha=0.3)
            fig.tight_layout()
            fig.savefig(args.out + '_plot.png', dpi=130)
            print('saved', args.out + '_plot.png')
        except ImportError as exc:
            print('matplotlib unavailable:', exc)


if __name__ == '__main__':
    main()
