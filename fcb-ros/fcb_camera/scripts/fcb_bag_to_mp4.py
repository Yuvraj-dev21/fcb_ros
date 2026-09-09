#!/usr/bin/env python3
"""Export the encoded FCB stream from a rosbag to an .mp4 without re-encoding.

    fcb_bag_to_mp4.py <bag_dir_or_file> [-o out.mp4] [--topic /fcb/image/ffmpeg]
                      [--keep-es] [--mkv]

What it does:
  1. Reads every ffmpeg_image_transport_msgs/FFMPEGPacket on the chosen topic
     (auto-picked when there is exactly one such topic).
  2. Skips packets until the first keyframe, then writes the Annex-B elementary
     stream (<out>.h265 / .h264) and a CSV with one row per frame: index, pts,
     ROS stamp, keyframe flag, size.
  3. Remuxes the elementary stream into an MP4 with `ffmpeg -c copy`, at the
     frame rate measured from the header stamps. With --mkv and mkvmerge
     installed, it also writes a Matroska file carrying the exact per-frame
     timestamps from the bag.

Requires: ros2 (rosbag2_py, rclpy), ffmpeg CLI. Optional: mkvmerge.
"""
import argparse
import csv
import os
import shutil
import statistics
import subprocess
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

PACKET_TYPE = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket"


def open_reader(path):
    storage_id = "mcap" if path.endswith(".mcap") else ""
    if os.path.isdir(path):
        for name in os.listdir(path):
            if name.endswith(".mcap"):
                storage_id = "mcap"
            elif name.endswith(".db3"):
                storage_id = "sqlite3"
    elif path.endswith(".db3"):
        storage_id = "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def pick_topic(reader, wanted):
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    packet_topics = [n for n, t in topics.items() if t == PACKET_TYPE]
    if wanted:
        if wanted not in topics:
            sys.exit(f"topic {wanted} not in bag; packet topics: {packet_topics}")
        if topics[wanted] != PACKET_TYPE:
            sys.exit(f"topic {wanted} is {topics[wanted]}, not {PACKET_TYPE}")
        return wanted
    if len(packet_topics) != 1:
        sys.exit(f"pick a topic with --topic, found: {packet_topics or 'none'}")
    return packet_topics[0]


def rate_from_stamps(stamps):
    if len(stamps) < 3:
        return 30.0, "30"
    deltas = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    if not deltas:
        return 30.0, "30"
    med = statistics.median(deltas)
    fps = 1.0 / med
    # snap to the common broadcast rates so the container gets a clean value
    for name, value in (("60000/1001", 59.94), ("60", 60.0), ("50", 50.0), ("30000/1001", 29.97),
                        ("30", 30.0), ("25", 25.0)):
        if abs(fps - value) < 0.15:
            return value, name
    if abs(fps - round(fps)) < 0.15:
        return float(round(fps)), str(int(round(fps)))
    return fps, f"{fps:.4f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("-o", "--output", help="output .mp4 (default: <bag>.mp4)")
    ap.add_argument("--topic", default="", help="packet topic (auto when unique)")
    ap.add_argument("--keep-es", action="store_true", help="keep the raw elementary stream file")
    ap.add_argument("--mkv", action="store_true", help="also write an .mkv with exact timestamps (needs mkvmerge)")
    args = ap.parse_args()

    bag = args.bag.rstrip("/")
    out_mp4 = args.output or (bag + ".mp4")
    base, _ = os.path.splitext(out_mp4)

    reader = open_reader(bag)
    topic = pick_topic(reader, args.topic)
    msg_type = get_message(PACKET_TYPE)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    codec = None
    es_path = None
    es = None
    stamps = []
    rows = []
    skipped = 0
    index = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, msg_type)
        if codec is None:
            codec = msg.encoding.split(";")[0].lower()
            if codec not in ("hevc", "h265", "h264"):
                sys.exit(f"unsupported codec in bag: {msg.encoding}")
            es_path = base + (".h265" if codec in ("hevc", "h265") else ".h264")
            es = open(es_path, "wb")
        if not rows and msg.flags == 0:
            skipped += 1
            continue  # need a keyframe to start decoding
        es.write(bytes(msg.data))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        stamps.append(stamp)
        rows.append((index, int(msg.pts), f"{stamp:.9f}", int(msg.flags != 0), len(msg.data)))
        index += 1
    if es:
        es.close()
    if not rows:
        sys.exit(f"no packets on {topic}")

    csv_path = base + "_frames.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["index", "pts", "stamp", "keyframe", "bytes"])
        w.writerows(rows)

    fps, fps_name = rate_from_stamps(stamps)
    duration = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
    total = sum(r[4] for r in rows)
    print(f"{len(rows)} frames ({skipped} skipped before first keyframe), codec {codec}, "
          f"{fps:.2f} fps, {duration:.1f} s, {total / 1e6:.1f} MB")

    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        sys.exit(f"ffmpeg not found; elementary stream left at {es_path}")
    demuxer = "hevc" if codec in ("hevc", "h265") else "h264"
    cmd = [ffmpeg, "-hide_banner", "-loglevel", "error", "-y", "-fflags", "+genpts",
           "-r", fps_name, "-f", demuxer, "-i", es_path,
           "-c", "copy", "-movflags", "+faststart", out_mp4]
    if codec in ("hevc", "h265"):
        cmd[-1:-1] = ["-tag:v", "hvc1"]  # lets QuickTime/Safari play the file
    subprocess.run(cmd, check=True)
    print(f"wrote {out_mp4}")
    print(f"wrote {csv_path}")

    if args.mkv:
        mkvmerge = shutil.which("mkvmerge")
        if not mkvmerge:
            print("mkvmerge not found (apt install mkvtoolnix); skipping .mkv", file=sys.stderr)
        else:
            ts_path = base + "_timestamps.txt"
            t0 = stamps[0]
            with open(ts_path, "w") as f:
                f.write("# timestamp format v2\n")
                for s in stamps:
                    f.write(f"{(s - t0) * 1000.0:.3f}\n")
            out_mkv = base + ".mkv"
            subprocess.run([mkvmerge, "-q", "-o", out_mkv, "--timestamps", f"0:{ts_path}", es_path], check=True)
            os.remove(ts_path)
            print(f"wrote {out_mkv} (exact per-frame timestamps)")

    if not args.keep_es:
        os.remove(es_path)
    else:
        print(f"kept {es_path}")


if __name__ == "__main__":
    main()
