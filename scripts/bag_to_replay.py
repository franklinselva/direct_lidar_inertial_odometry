#!/usr/bin/env python3
"""Convert a ROS1 bag (PointCloud2 + Imu) into the file-replay format consumed
by the dlio_live_mapping example: per-scan binary PCD files named <stamp>.pcd
(carrying x,y,z,intensity,t) plus an IMU CSV (stamp,wx,wy,wz,ax,ay,az).

Pure Python via the `rosbags` library — no ROS install required.

Usage:
  bag_to_replay.py BAG OUT_DIR [--lidar TOPIC] [--imu TOPIC] [--max-scans N]
"""
import argparse
import struct
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

# sensor_msgs/PointField datatype -> numpy dtype
PF = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}

PCD_HEADER = (
    "# .PCD v0.7 - Point Cloud Data file format\n"
    "VERSION 0.7\n"
    "FIELDS x y z intensity t\n"
    "SIZE 4 4 4 4 4\n"
    "TYPE F F F F U\n"
    "COUNT 1 1 1 1 1\n"
    "WIDTH {n}\nHEIGHT 1\n"
    "VIEWPOINT 0 0 0 1 0 0 0\n"
    "POINTS {n}\nDATA binary\n"
)

OUT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                      ('intensity', '<f4'), ('t', '<u4')])


def cloud_to_array(m):
    """View the raw PointCloud2 buffer as a structured array, by field offset."""
    names, formats, offsets = [], [], []
    for f in m.fields:
        if f.name not in ('x', 'y', 'z', 'intensity', 't'):
            continue
        names.append(f.name)
        formats.append(PF[f.datatype])
        offsets.append(f.offset)
    dt = np.dtype({'names': names, 'formats': formats,
                   'offsets': offsets, 'itemsize': m.point_step})
    n = m.width * m.height
    arr = np.frombuffer(bytes(m.data), dtype=dt, count=n)

    out = np.empty(n, dtype=OUT_DTYPE)
    out['x'] = arr['x']; out['y'] = arr['y']; out['z'] = arr['z']
    out['intensity'] = arr['intensity'] if 'intensity' in names else 0.0
    out['t'] = arr['t'] if 't' in names else 0
    # drop non-finite points
    finite = np.isfinite(out['x']) & np.isfinite(out['y']) & np.isfinite(out['z'])
    return out[finite]


def write_pcd(path, arr):
    with open(path, 'wb') as f:
        f.write(PCD_HEADER.format(n=len(arr)).encode())
        f.write(arr.tobytes())


def stamp_sec(hdr):
    return hdr.stamp.sec + hdr.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('out_dir')
    ap.add_argument('--lidar', default='/robot/lidar')
    ap.add_argument('--imu', default='/robot/imu')
    ap.add_argument('--max-scans', type=int, default=0)
    args = ap.parse_args()

    out = Path(args.out_dir)
    pcd_dir = out / 'pcd'
    pcd_dir.mkdir(parents=True, exist_ok=True)

    n_scans = 0
    n_imu = 0
    with AnyReader([Path(args.bag)]) as r, open(out / 'imu.csv', 'w') as imuf:
        imuf.write('# stamp,wx,wy,wz,ax,ay,az\n')
        conns = [c for c in r.connections if c.topic in (args.lidar, args.imu)]
        for conn, _ts, raw in r.messages(connections=conns):
            m = r.deserialize(raw, conn.msgtype)
            if conn.topic == args.imu:
                s = stamp_sec(m.header)
                w, a = m.angular_velocity, m.linear_acceleration
                imuf.write(f"{s:.9f},{w.x:.9f},{w.y:.9f},{w.z:.9f},"
                           f"{a.x:.9f},{a.y:.9f},{a.z:.9f}\n")
                n_imu += 1
            elif conn.topic == args.lidar:
                if args.max_scans and n_scans >= args.max_scans:
                    continue
                arr = cloud_to_array(m)
                write_pcd(pcd_dir / f"{stamp_sec(m.header):.9f}.pcd", arr)
                n_scans += 1

    print(f"wrote {n_scans} scans -> {pcd_dir}")
    print(f"wrote {n_imu} IMU samples -> {out / 'imu.csv'}")


if __name__ == '__main__':
    main()
