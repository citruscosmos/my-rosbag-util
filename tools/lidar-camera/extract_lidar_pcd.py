#!/usr/bin/env python3
"""bag_converter でデコード済みの seyond_points(PointCloud2) を sensor 別に PCD 保存する。

PCD は binary 形式 (FIELDS x y z intensity)。ファイル名は header.stamp のナノ秒。
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

POINTS_MAP = {
    "/sensing/lidar/front/seyond_points": "lidar_front",
    "/sensing/lidar/left/seyond_points": "lidar_left",
    "/sensing/lidar/rear/seyond_points": "lidar_rear",
    "/sensing/lidar/right/seyond_points": "lidar_right",
}

# sensor_msgs/PointField datatype -> numpy dtype
PF_TO_NP = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
            5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def cloud_to_xyzi(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 -> (N,4) float32 [x,y,z,intensity], 非有限点は除去。"""
    names = {f.name: f for f in msg.fields}
    fields = []
    for f in msg.fields:
        np_t = PF_TO_NP.get(f.datatype)
        if np_t is None:
            continue
        fields.append((f.name, np_t, f.offset))
    dt = np.dtype({
        "names": [n for n, _, _ in fields],
        "formats": [t for _, t, _ in fields],
        "offsets": [o for _, _, o in fields],
        "itemsize": msg.point_step,
    })
    n = msg.width * msg.height
    arr = np.frombuffer(bytes(msg.data), dtype=dt, count=n)
    x = arr["x"].astype(np.float32)
    y = arr["y"].astype(np.float32)
    z = arr["z"].astype(np.float32)
    if "intensity" in names:
        inten = arr["intensity"].astype(np.float32)
    else:
        inten = np.zeros(n, dtype=np.float32)
    out = np.stack([x, y, z, inten], axis=1)
    finite = np.isfinite(out).all(axis=1)
    return out[finite]


def write_pcd_binary(path: Path, pts: np.ndarray) -> None:
    n = pts.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.ascontiguousarray(pts, dtype=np.float32).tobytes())


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mcap")
    ap.add_argument("out_root")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--start", type=float, default=None, help="開始時刻(UNIX秒)")
    ap.add_argument("--end", type=float, default=None, help="終了時刻(UNIX秒)")
    args = ap.parse_args()

    out_root = Path(args.out_root)
    counts = {name: 0 for name in POINTS_MAP.values()}
    collisions = 0
    total = 0
    t0 = time.time()

    start_ns = int(args.start * 1e9) if args.start is not None else None
    end_ns = int(args.end * 1e9) if args.end is not None else None

    with open(args.mcap, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(
                topics=list(POINTS_MAP.keys()), start_time=start_ns, end_time=end_ns):
            name = POINTS_MAP.get(channel.topic)
            if name is None:
                continue
            if args.limit and counts[name] >= args.limit:
                if all(counts[n] >= args.limit for n in POINTS_MAP.values()):
                    break
                continue
            msg = deserialize_message(message.data, PointCloud2)
            pts = cloud_to_xyzi(msg)
            stamp = msg.header.stamp
            fname = f"{stamp.sec}{stamp.nanosec:09d}.pcd"
            dst = out_root / name / fname
            if dst.exists():
                collisions += 1
                dst = out_root / name / f"{stamp.sec}{stamp.nanosec:09d}_{counts[name]}.pcd"
            write_pcd_binary(dst, pts)
            counts[name] += 1
            total += 1
            if total % 1000 == 0:
                dt = time.time() - t0
                print(f"[lidar] {total} / {dt:.0f}s "
                      + " ".join(f"{n}:{c}" for n, c in counts.items()), flush=True)

    dt = time.time() - t0
    print(f"[lidar] DONE total={total} collisions={collisions} elapsed={dt:.0f}s", flush=True)
    for n, c in counts.items():
        print(f"  {n}: {c}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
