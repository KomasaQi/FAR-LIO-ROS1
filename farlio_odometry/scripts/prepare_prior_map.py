#!/usr/bin/env python3
"""Prepare a prior map for FAR-LIO localization from a large global PCD.

FAR-LIO keeps the prior map in a voxel hash map and indexes it with float32
positions / int16 voxel keys, so a 35M-point global map (HitchOpen CICV) is both
too large to load and too far from the origin. This script:

  1. reads an optional trajectory PCD (x y z intensity roll pitch yaw time) to get
     the driven area,
  2. crops the global map to the trajectory bounding box (+margin) and a z band,
  3. voxel-downsamples it,
  4. writes a binary PCD (x y z intensity) in the same frame.

It also prints the first trajectory pose, which is the initial pose to put into
the FAR-LIO localization configuration.

Usage:
  python3 prepare_prior_map.py --map GlobalMap.pcd --trajectory transformations.pcd \
      --out pix_cicv_map.pcd --margin 40 --voxel 0.5 --z-min -20 --z-max 20
"""
import argparse
import sys

import numpy as np


def pcd_header(path):
    with open(path, 'rb') as f:
        lines = []
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError('invalid PCD')
            lines.append(line)
            if line.strip() == b'DATA binary':
                break
    return b''.join(lines)


def read_pcd_xyz(path):
    raw = open(path, 'rb').read()
    end = raw.find(b'DATA binary\n')
    if end < 0:
        raise RuntimeError('only binary PCD is supported')
    offset = end + len(b'DATA binary\n')
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')])
    return np.frombuffer(raw[offset:], dtype=dt), offset


def read_trajectory(path):
    raw = open(path, 'rb').read()
    end = raw.find(b'DATA binary\n')
    offset = end + len(b'DATA binary\n')
    dt = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4'),
                   ('roll', '<f4'), ('pitch', '<f4'), ('yaw', '<f4'), ('time', '<f8')])
    return np.frombuffer(raw[offset:], dtype=dt)


def write_pcd_xyzi(path, points):
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\n'
        'VERSION 0.7\n'
        'FIELDS x y z intensity\n'
        'SIZE 4 4 4 4\n'
        'TYPE F F F F\n'
        'COUNT 1 1 1 1\n'
        f'WIDTH {len(points)}\n'
        'HEIGHT 1\n'
        'VIEWPOINT 0 0 0 1 0 0 0\n'
        f'POINTS {len(points)}\n'
        'DATA binary\n')
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(points.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--trajectory', default='')
    ap.add_argument('--out', required=True)
    ap.add_argument('--margin', type=float, default=40.0)
    ap.add_argument('--voxel', type=float, default=0.5)
    ap.add_argument('--z-min', type=float, default=-20.0)
    ap.add_argument('--z-max', type=float, default=20.0)
    ap.add_argument('--chunk', type=int, default=4_000_000)
    ap.add_argument('--x-min', type=float, default=None)
    ap.add_argument('--x-max', type=float, default=None)
    ap.add_argument('--y-min', type=float, default=None)
    ap.add_argument('--y-max', type=float, default=None)
    args = ap.parse_args()

    traj = read_trajectory(args.trajectory) if args.trajectory else None
    if traj is not None and len(traj):
        xmin, xmax = traj['x'].min() - args.margin, traj['x'].max() + args.margin
        ymin, ymax = traj['y'].min() - args.margin, traj['y'].max() + args.margin
        zmin = max(traj['z'].min() - 30.0, args.z_min)
        zmax = min(traj['z'].max() + 10.0, args.z_max)
        p0 = traj[0]
        print('trajectory poses:', len(traj))
        print('first pose: x=%.4f y=%.4f z=%.4f roll=%.6f pitch=%.6f yaw=%.6f' % (
            p0['x'], p0['y'], p0['z'], p0['roll'], p0['pitch'], p0['yaw']))
        print('crop: x [%.1f, %.1f] y [%.1f, %.1f] z [%.1f, %.1f]' % (
            xmin, xmax, ymin, ymax, zmin, zmax))
    else:
        xmin = ymin = -1e30
        xmax = ymax = 1e30
        zmin, zmax = args.z_min, args.z_max
        print('no trajectory: using the whole map, z from args')
    # Explicit bounding-box overrides (for maps without a trajectory file).
    if args.x_min is not None: xmin = args.x_min
    if args.x_max is not None: xmax = args.x_max
    if args.y_min is not None: ymin = args.y_min
    if args.y_max is not None: ymax = args.y_max
    print('final crop: x [%.1f, %.1f] y [%.1f, %.1f] z [%.1f, %.1f]' % (
        xmin, xmax, ymin, ymax, zmin, zmax))

    mm = np.memmap(args.map, dtype=np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                             ('i', '<f4')]),
                   mode='r', offset=len(pcd_header(args.map)))
    total = len(mm)
    print('global map points:', total)

    # Pass 1: crop.
    chunks = []
    kept = 0
    for start in range(0, total, args.chunk):
        c = np.asarray(mm[start:start + args.chunk])
        m = ((c['x'] >= xmin) & (c['x'] <= xmax) &
             (c['y'] >= ymin) & (c['y'] <= ymax) &
             (c['z'] >= zmin) & (c['z'] <= zmax) &
             np.isfinite(c['x']) & np.isfinite(c['y']) & np.isfinite(c['z']))
        c = c[m]
        kept += len(c)
        if len(c):
            chunks.append(c)
    pts = np.concatenate(chunks) if chunks else np.zeros(0, dtype=mm.dtype)
    print('points after crop: %d' % kept)

    # Pass 2: voxel-downsample with the first point per voxel (numpy sort).
    voxel = float(args.voxel)
    if voxel > 0 and len(pts):
        inv = 1.0 / voxel
        ix = np.floor(pts['x'] * inv).astype(np.int64)
        iy = np.floor(pts['y'] * inv).astype(np.int64)
        iz = np.floor(pts['z'] * inv).astype(np.int64)
        keys = (ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791)
        _, first = np.unique(keys, return_index=True)
        pts = pts[np.sort(first)]
    print('points after %.2f m voxel: %d' % (voxel, len(pts)))

    write_pcd_xyzi(args.out, pts)
    print('saved', args.out)


if __name__ == '__main__':
    sys.exit(main())
