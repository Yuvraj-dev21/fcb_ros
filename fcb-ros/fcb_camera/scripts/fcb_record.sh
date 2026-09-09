#!/usr/bin/env bash
# Record the encoded FCB stream plus its metadata into a rosbag.
#
#   fcb_record.sh [bag_name] [extra ros2 bag record args...]
#
# Only the packet topic, CameraInfo and the camera state are recorded, so a
# 1080p60 H.265 stream at 8 Mbit/s costs about 1 GB per 17 minutes.
set -euo pipefail
NS="${FCB_NAMESPACE:-fcb}"
NAME="${1:-fcb-$(date +%Y%m%d-%H%M%S)}"
shift || true
exec ros2 bag record -o "$NAME" \
  "/$NS/image/ffmpeg" "/$NS/camera_info" "/$NS/state" "$@"
