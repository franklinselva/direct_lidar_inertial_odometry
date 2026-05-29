#!/usr/bin/env python3
"""Render a snapshot of a dense map PCD with the trajectory overlaid, to confirm
the LiDAR data overlays seamlessly. Produces perspective + top-down + side views.

Usage: snapshot_map.py DENSE_MAP.pcd TRAJECTORY.txt OUT.png [--max-points N]
"""
import argparse
import struct
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def read_pcd_xyz_intensity(path):
    """Minimal binary/ascii PCD reader for x,y,z[,intensity]."""
    with open(path, 'rb') as f:
        fields, sizes, types, counts = [], [], [], []
        n = 0; data_kind = None
        while True:
            line = f.readline().decode('ascii', 'replace').strip()
            if line.startswith('FIELDS'): fields = line.split()[1:]
            elif line.startswith('SIZE'): sizes = list(map(int, line.split()[1:]))
            elif line.startswith('TYPE'): types = line.split()[1:]
            elif line.startswith('COUNT'): counts = list(map(int, line.split()[1:]))
            elif line.startswith('POINTS'): n = int(line.split()[1])
            elif line.startswith('DATA'):
                data_kind = line.split()[1]; break
        np_t = {('F', 4): '<f4', ('F', 8): '<f8', ('U', 4): '<u4', ('U', 2): '<u2',
                ('U', 1): '<u1', ('I', 4): '<i4', ('I', 2): '<i2', ('I', 1): '<i1'}
        names, formats, offsets = [], [], []
        off = 0
        for fn, sz, ty, ct in zip(fields, sizes, types, counts):
            names.append(fn); formats.append(np_t[(ty, sz)]); offsets.append(off)
            off += sz * ct
        dt = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': off})
        if data_kind == 'binary':
            arr = np.frombuffer(f.read(n * off), dtype=dt, count=n)
        else:
            arr = np.loadtxt(f, max_rows=n)
            d = np.empty(n, dtype=dt)
            for i, nm in enumerate(names): d[nm] = arr[:, i]
            arr = d
    xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=1)
    inten = arr['intensity'] if 'intensity' in names else arr['z']
    return xyz.astype(np.float64), np.asarray(inten, np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd'); ap.add_argument('traj'); ap.add_argument('out')
    ap.add_argument('--max-points', type=int, default=400000)
    args = ap.parse_args()

    xyz, inten = read_pcd_xyz_intensity(args.pcd)
    finite = np.isfinite(xyz).all(1)
    xyz, inten = xyz[finite], inten[finite]
    if len(xyz) > args.max_points:
        idx = np.random.default_rng(0).choice(len(xyz), args.max_points, replace=False)
        xyz, inten = xyz[idx], inten[idx]

    traj = np.loadtxt(args.traj)
    tp = traj[:, 1:4] if traj.ndim == 2 else traj[None, 1:4]

    c = np.clip((inten - np.percentile(inten, 2)) /
                (np.percentile(inten, 98) - np.percentile(inten, 2) + 1e-9), 0, 1)

    fig = plt.figure(figsize=(18, 6))

    ax = fig.add_subplot(1, 3, 1, projection='3d')
    ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=c, cmap='viridis', s=0.5, lw=0)
    ax.plot(tp[:, 0], tp[:, 1], tp[:, 2], 'r-', lw=1.5)
    ax.set_title(f'perspective  ({len(xyz)} pts shown)')

    ax = fig.add_subplot(1, 3, 2)
    ax.scatter(xyz[:, 0], xyz[:, 1], c=c, cmap='viridis', s=0.5, lw=0)
    ax.plot(tp[:, 0], tp[:, 1], 'r-', lw=1.2)
    ax.set_title('top-down (x-y)'); ax.set_aspect('equal'); ax.set_xlabel('x[m]'); ax.set_ylabel('y[m]')

    ax = fig.add_subplot(1, 3, 3)
    ax.scatter(xyz[:, 0], xyz[:, 2], c=c, cmap='viridis', s=0.5, lw=0)
    ax.plot(tp[:, 0], tp[:, 2], 'r-', lw=1.2)
    ax.set_title('side (x-z)'); ax.set_aspect('equal'); ax.set_xlabel('x[m]'); ax.set_ylabel('z[m]')

    plt.tight_layout()
    plt.savefig(args.out, dpi=120)
    print(f"saved {args.out}  (map pts total render={len(xyz)})")


if __name__ == '__main__':
    main()
