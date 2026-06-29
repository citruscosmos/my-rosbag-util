#!/usr/bin/env python3
"""MCAP の camera8-11 CompressedImage(jpeg) を sensor 別ディレクトリに JPG 保存する。

ファイル名は header.stamp のナノ秒(エポック)。 {sec}{nanosec:09d}.jpg
"""
import argparse
import sys
import time
from pathlib import Path

from mcap.reader import make_reader
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

def build_cam_map(cam_ids):
    return {
        f"/sensing/camera/camera{i}/image_raw/compressed": f"camera{i}"
        for i in cam_ids
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("mcap")
    ap.add_argument("out_root")
    ap.add_argument("--cams", default="8,9,10,11", help="カメラ番号(カンマ区切り)")
    ap.add_argument("--limit", type=int, default=0, help="各カメラ最大枚数(0=無制限/検証用)")
    ap.add_argument("--start", type=float, default=None, help="開始時刻(UNIX秒)")
    ap.add_argument("--end", type=float, default=None, help="終了時刻(UNIX秒)")
    args = ap.parse_args()

    global CAM_MAP
    CAM_MAP = build_cam_map([int(x) for x in args.cams.split(",")])

    out_root = Path(args.out_root)
    counts = {name: 0 for name in CAM_MAP.values()}
    collisions = 0
    t0 = time.time()
    total = 0

    start_ns = int(args.start * 1e9) if args.start is not None else None
    end_ns = int(args.end * 1e9) if args.end is not None else None

    with open(args.mcap, "rb") as f:
        reader = make_reader(f)
        for _schema, channel, message in reader.iter_messages(
                topics=list(CAM_MAP.keys()), start_time=start_ns, end_time=end_ns):
            name = CAM_MAP.get(channel.topic)
            if name is None:
                continue
            if args.limit and counts[name] >= args.limit:
                # 全カメラが上限に達したら終了
                if all(counts[n] >= args.limit for n in CAM_MAP.values()):
                    break
                continue
            msg = deserialize_message(message.data, CompressedImage)
            stamp = msg.header.stamp
            fname = f"{stamp.sec}{stamp.nanosec:09d}.jpg"
            dst = out_root / name / fname
            if dst.exists():
                collisions += 1
                dst = out_root / name / f"{stamp.sec}{stamp.nanosec:09d}_{counts[name]}.jpg"
            with open(dst, "wb") as g:
                g.write(bytes(msg.data))
            counts[name] += 1
            total += 1
            if total % 2000 == 0:
                dt = time.time() - t0
                print(f"[cam] {total} 枚 / {dt:.0f}s "
                      + " ".join(f"{n}:{c}" for n, c in counts.items()), flush=True)

    dt = time.time() - t0
    print(f"[cam] DONE total={total} collisions={collisions} elapsed={dt:.0f}s", flush=True)
    for n, c in counts.items():
        print(f"  {n}: {c}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
