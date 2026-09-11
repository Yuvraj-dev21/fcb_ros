# fcb-ros — ROS 2 driver for the Sony FCB-EV9520L block camera

A ROS 2 Humble driver for the Sony FCB-EV9520L (30x optical zoom, 1080p60)
on its USB interface board. It captures YUYV 1080p60 with zero-copy V4L2,
encodes to H.265 (H.264 optional) on the GPU or CPU, and publishes the
encoded stream as `ffmpeg_image_transport` packets. Bags stay small: about
**1 GB per 17 minutes** at the default 8 Mbit/s cap, versus roughly 350 GB
for the same period of raw YUYV. Camera control (zoom, focus, IR cut filter,
exposure, white balance, picture settings, presets) is exposed as typed ROS
services over VISCA.

```
 camera ──USB──▶ V4L2 mmap ──▶ memcpy ──▶ bounded queue ──▶ encoder ──▶ FFMPEGPacket ──▶ /fcb/image/ffmpeg
 (NeoHD)          capture thread                            encode thread    + CameraInfo  ──▶ /fcb/camera_info
        ──USB──▶ /dev/ttyACM0 (VISCA) ◀── services, 2 Hz state poll ──────────────────────▶ /fcb/state
```

## Packages

| package | contents |
|---|---|
| `fcb_interfaces` | `CameraState` message; services `SetZoom`, `SetFocus`, `SetIcr`, `SetExposure`, `SetWhiteBalance`, `SetPicture`, `Preset`, `ViscaCommand` |
| `fcb_camera` | driver node (`fcb_camera_node`, also a composable component), launch files, parameter YAML, calibration stub, udev rules + installer, bag-to-MP4 exporter, record wrapper, fake VISCA camera, unit tests |



## Installation

Dependencies (Ubuntu 22.04 / ROS 2 Humble):

```bash
sudo apt install ros-humble-camera-info-manager ros-humble-image-transport \
     ros-humble-ffmpeg-image-transport ros-humble-ffmpeg-image-transport-msgs \
     ros-humble-ffmpeg-encoder-decoder ros-humble-rosbag2 \
     libavcodec-dev libavutil-dev libswscale-dev v4l-utils ffmpeg
# Jetson hardware encoder backend (needs the GStreamer dev headers):
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

If the `ros-humble-ffmpeg-*` packages cannot be installed (as on the
development laptop, which has no sudo), clone them next to the driver and
build them from source; that is what the checkouts of
`ffmpeg_image_transport_msgs`, `ffmpeg_encoder_decoder` and
`ffmpeg_image_transport` in the workspace root are for.

Build:

```bash
cd ~/fcb-ros                                  # workspace root
source /opt/ros/humble/setup.bash
# only needed when building the ffmpeg transport from source:
colcon build --packages-select ffmpeg_image_transport_msgs ffmpeg_encoder_decoder \
             ffmpeg_image_transport --cmake-args -DBUILD_TESTING=OFF
colcon build --packages-select fcb_interfaces fcb_camera
source install/setup.bash
```

CMake prints `GStreamer dev packages not found; building without the
GStreamer backend` when the Jetson backend is skipped. Everything else still
works through libav.

Stable device names and USB-reset permission (once, camera plugged in):

```bash
ros2 run fcb_camera fcb_udev_setup.sh --dry-run   # shows the rules it would write
ros2 run fcb_camera fcb_udev_setup.sh             # installs them with sudo
# or copy the shipped file:
sudo cp $(ros2 pkg prefix fcb_camera)/share/fcb_camera/udev/99-fcb-camera.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

Afterwards `/dev/fcb_video` and `/dev/fcb_visca` point at the board and the
driver uses them without probing.

## Running

```bash
ros2 launch fcb_camera fcb_camera.launch.py                     # autodetect everything, H.265, 8 Mbit/s
ros2 launch fcb_camera fcb_camera.launch.py codec:=h264 bit_rate:=4000000
ros2 launch fcb_camera fcb_camera.launch.py video_device:=/dev/video3 visca_port:=/dev/ttyACM0
ros2 launch fcb_camera fcb_camera.launch.py backend:=libav encoder_name:=hevc_nvenc
ros2 launch fcb_camera fcb_camera.launch.py params_file:=/path/to/my.yaml
```

Launch arguments: `namespace` (default `fcb`), `params_file`, `video_device`,
`visca_port`, `backend`, `codec`, `encoder_name`, `bit_rate`, `fps`, `width`,
`height`, `camera_info_url`, `log_level`.

Expected log on a good start:

```
VISCA camera on /dev/fcb_visca @ 9600 baud (model 0x0711, rom 0x0103)
video autodetect: using udev symlink /dev/fcb_video (USB3 NeoHD: USB3Neo 012099)
encoder libav:hevc_nvenc ready (hevc;yuv420p;bgr8;bgr8), 8.0 Mbit/s cap, gop 30, quality 23
streaming 1920x1080 YUYV @ 59.94 fps from /dev/fcb_video ("USB3 NeoHD: USB3Neo 012099")
```

### Topics

All names are relative to the namespace (`/fcb` by default).

| topic | type | notes |
|---|---|---|
| `image/ffmpeg` | `ffmpeg_image_transport_msgs/FFMPEGPacket` | one packet per frame; `flags`=1 on keyframes; `encoding` e.g. `hevc;yuv420p;bgr8;bgr8`. **This is the topic to record.** |
| `camera_info` | `sensor_msgs/CameraInfo` | same stamp as the packet; `fx`,`fy` multiplied by the current zoom ratio when `camera_info.scale_with_zoom` is true |
| `state` | `fcb_interfaces/CameraState` | lens, image settings, link status and stream statistics at `state_rate_hz` |

There is deliberately **no raw image topic**. To get `sensor_msgs/Image`
locally:

```bash
ros2 launch fcb_camera view.launch.py            # republishes to /fcb/image/decoded (bgr8)
ros2 run rqt_image_view rqt_image_view /fcb/image/decoded
```

or pick the `ffmpeg` transport for `/fcb/image` in any image_transport
consumer. Decoding prefers `hevc_cuvid`/`h264_cuvid` when available and falls
back to the software decoder.

### `CameraState` fields

`control_connected`, `visca_port`, `visca_baud`, `model_id`, `rom_version`,
`zoom_position` (raw, 0..0x4000 optical), `zoom_ratio` (1..30 optical, up to
360 with digital zoom), `digital_zoom_enabled`, `focus_position`,
`focus_mode`, `icr_mode` (`day`/`night`/`night_color`), `auto_icr`,
`exposure_mode`, `shutter`, `iris`, `gain`, `white_balance_mode`,
`stabilizer_on`, `power_on`, `streaming`, `video_status`, `video_device`,
`width`, `height`, `capture_fps`, `publish_fps`, `bitrate_mbps`,
`frames_captured`, `frames_dropped`, `encoder`.

`video_status` is one of `streaming`, `stalled`, `reopening in N s (attempt
K)`, `usb reset`, `no device`.

### Services

| service | request highlights |
|---|---|
| `set_zoom` | `mode`: `ratio` (`value` 1..30, up to 360 with digital zoom), `position` (raw), `tele`/`wide` (`speed` 0..7, then `stop`), `stop`, `digital` (`value` ≥1 on). `wait: true` blocks until the lens arrives. |
| `set_focus` | `mode`: `auto`, `manual`, `one_push`, `position` (raw 0x1000 far .. 0xF000 near), `far`/`near` (`speed`), `stop` |
| `set_icr` | `mode`: `day`, `night`, `night_color`, `auto`, `auto_color`, `manual`; aliases `rgb`/`color`, `ir`/`bw`/`mono`. `auto_threshold` 0..255 |
| `set_exposure` | `mode`: `full_auto`, `manual`, `shutter_priority`, `iris_priority`, `bright`; `shutter`, `iris`, `gain`, `gain_limit`, `brightness`, `exposure_compensation` (-1 = unchanged); `exposure_compensation_enable`, `backlight_compensation`, `slow_shutter_auto` (`on`/`off`/`""`) |
| `set_white_balance` | `mode`: `auto`, `indoor`, `outdoor`, `one_push`, `atw`, `manual`, `outdoor_auto`, `sodium_auto`, `sodium`, `sodium_outdoor_auto`; `trigger_one_push`; `red_gain`, `blue_gain` |
| `set_picture` | `stabilizer`, `defog` (+`defog_level` 1..3), `noise_reduction` 0..5, `aperture` 0..15, `flip`, `mirror`, `freeze`, `digital_zoom`, `power` |
| `preset` | `action`: `set`, `recall`, `reset`; `slot` 0..15 |
| `visca` | raw packet: `hex: "09 04 47"` or `payload` bytes; `inquiry: true` returns `reply_hex` |
| `reconnect_visca` | `std_srvs/Trigger`, re-runs the serial sweep |

Every response carries `success` and `message`; the lens services also return
the position read back after the move.

```bash
ros2 service call /fcb/set_zoom fcb_interfaces/srv/SetZoom "{mode: ratio, value: 12.0, wait: true}"
ros2 service call /fcb/set_zoom fcb_interfaces/srv/SetZoom "{mode: tele, speed: 4}"    # ... then {mode: stop}
ros2 service call /fcb/set_icr fcb_interfaces/srv/SetIcr "{mode: night}"
ros2 service call /fcb/set_focus fcb_interfaces/srv/SetFocus "{mode: one_push}"
ros2 service call /fcb/set_exposure fcb_interfaces/srv/SetExposure "{mode: shutter_priority, shutter: 14}"
ros2 service call /fcb/set_picture fcb_interfaces/srv/SetPicture "{stabilizer: 'on', defog: 'on', defog_level: 2}"
ros2 service call /fcb/preset fcb_interfaces/srv/Preset "{action: set, slot: 1}"
ros2 service call /fcb/visca fcb_interfaces/srv/ViscaCommand "{hex: '09 00 02', inquiry: true}"   # version
ros2 topic echo /fcb/state
```

## Parameters

Defaults live in `fcb_camera/config/fcb_camera.yaml`.

| parameter | default | meaning |
|---|---|---|
| `video.device` | `""` | V4L2 node; empty = `/dev/fcb_video` if present, else the first capture node whose card name matches `video.device_hints` |
| `video.device_hints` | `[NeoHD, FCB, Harrier]` | card-name substrings that identify the camera board |
| `video.width` / `video.height` / `video.fps` | 1920 / 1080 / 60.0 | 59.94 is understood |
| `video.pixel_format` | `YUYV` | the only format the board offers |
| `video.buffers` | 6 | V4L2 mmap buffers |
| `video.timestamp_source` | `v4l2` | kernel capture timestamp mapped to ROS time; `node` = time of dequeue |
| `frame_id` / `camera_name` | `fcb_optical_frame` / `fcb_ev9520l` | |
| `camera_info_url` | `package://fcb_camera/config/fcb_ev9520l_1080p.yaml` | placeholder calibration at 1x |
| `camera_info.scale_with_zoom` | true | scale fx, fy by the optical zoom ratio |
| `encoder.backend` | `auto` | `auto` = GStreamer `nvv4l2h265enc` when present (Jetson), else libav; or `libav` / `gstreamer` |
| `encoder.codec` | `hevc` | or `h264` |
| `encoder.name` | `""` | libav encoder (`hevc_nvenc`, `libx265`, …) or GStreamer element; empty = automatic |
| `encoder.bit_rate` | 8000000 | bit/s cap |
| `encoder.gop_size` | 30 | keyframe every 0.5 s at 60 fps |
| `encoder.max_b_frames` | 0 | no reordering, lowest latency |
| `encoder.quality` | 23 | crf / cq target, lower is better; -1 = bitrate only |
| `encoder.preset` | `""` | `superfast` (x265), `veryfast` (x264), `p4` (nvenc) |
| `encoder.av_options` | `""` | extra libav options `key:value,key:value` |
| `encoder.gst_pipeline` | `""` | full GStreamer override; must contain `appsrc name=fcbsrc` and `appsink name=fcbsink`; `{width} {height} {fps_num} {fps_den} {bitrate} {gop}` placeholders |
| `encoder.threads` | 0 | software encoder threads |
| `encoder.queue_depth` | 4 | frames buffered before the encoder; beyond that frames are dropped and counted |
| `visca.port` | `""` | empty = probe `/dev/fcb_visca`, `/dev/ttyACM*`, `/dev/ttyUSB*`, `/dev/ttyTHS*` |
| `visca.baud` | 0 | 0 = try 9600, 38400, 115200 |
| `visca.address` / `visca.timeout` | 1 / 0.5 s | |
| `visca.required` | false | refuse to start without the control link |
| `visca.allow_digital_zoom` | false | when false the driver switches the camera's digital zoom off at connect so `tele` stops at 30x |
| `state_rate_hz` | 2.0 | state publish and lens poll rate |
| `startup.icr_mode`, `startup.focus_mode`, `startup.exposure_mode`, `startup.stabilizer`, `startup.digital_zoom`, `startup.zoom_ratio` | unset | applied once after connecting |

## Recording and export

```bash
ros2 run fcb_camera fcb_record.sh flight1         # records image/ffmpeg + camera_info + state
ros2 run fcb_camera fcb_bag_to_mp4.py flight1     # flight1.mp4 + flight1_frames.csv, no re-encoding
ros2 run fcb_camera fcb_bag_to_mp4.py flight1 --mkv   # also an .mkv with the exact per-frame timestamps
```

The MP4 is written at the nominal frame rate measured from the stamps; if
frames were dropped during recording the MP4 plays slightly fast in those
stretches. The CSV has the ROS stamp of every frame and the MKV keeps the
real timing.

Playback: `ros2 bag play flight1`, then `view.launch.py` decodes
`/fcb/image/ffmpeg` back to images.

## Jetson Orin: dependencies, build, run

Tested design target: Jetson Orin Nano / NX, JetPack 6 (Ubuntu 22.04) with ROS 2 Humble.

### 1. Dependencies

```bash
# ROS 2 Humble (skip if installed): https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
sudo apt update
sudo apt install -y \
  ros-humble-ros-base ros-humble-camera-info-manager ros-humble-image-transport \
  ros-humble-ffmpeg-image-transport ros-humble-ffmpeg-image-transport-msgs \
  ros-humble-ffmpeg-encoder-decoder ros-humble-cv-bridge \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-mcap ros-humble-rqt-image-view \
  python3-colcon-common-extensions python3-rosdep python3-serial \
  libavcodec-dev libavutil-dev libswscale-dev ffmpeg v4l-utils \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
# JetPack already provides the NVIDIA GStreamer plugins; confirm the hardware encoder is visible:
gst-inspect-1.0 nvv4l2h265enc | head -3
# device access
sudo usermod -aG video,dialout,plugdev $USER      # log out and in again
```

If `ros-humble-ffmpeg-image-transport` is not available for arm64 on your
mirror, clone the three repos into the workspace and build them from source
(same commands as below; that is what this workspace does):

```bash
cd ~/fcb-ros
git clone https://github.com/ros-misc-utilities/ffmpeg_image_transport_msgs.git
git clone https://github.com/ros-misc-utilities/ffmpeg_encoder_decoder.git
git clone https://github.com/ros-misc-utilities/ffmpeg_image_transport.git
```

### 2. Build

```bash
# copy this workspace to the Orin, e.g. rsync -a --exclude build --exclude install --exclude log ~/fcb-ros nvidia@orin:
cd ~/fcb-ros
source /opt/ros/humble/setup.bash
rosdep install --from-paths fcb-ros --ignore-src -r -y
colcon build --packages-select ffmpeg_image_transport_msgs ffmpeg_encoder_decoder ffmpeg_image_transport \
             --cmake-args -DBUILD_TESTING=OFF        # only when built from source
colcon build --packages-select fcb_interfaces fcb_camera --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

The configure step must **not** print `building without the GStreamer
backend`; if it does, the GStreamer dev packages are missing and the driver
would fall back to CPU `libx265`, which cannot hold 1080p60 on an Orin.

### 3. Stable device names (once)

```bash
ros2 run fcb_camera fcb_udev_setup.sh      # camera plugged in; installs /etc/udev/rules.d/99-fcb-camera.rules
```

### 4. Run

```bash
source ~/fcb-ros/install/setup.bash
ros2 launch fcb_camera fcb_camera.launch.py                 # auto: nvv4l2h265enc via GStreamer, 8 Mbit/s
ros2 launch fcb_camera fcb_camera.launch.py backend:=gstreamer codec:=h264   # H.264 hardware encoder
ros2 run fcb_camera fcb_record.sh flight1                    # record
```

Expected log line: `encoder gstreamer:nvv4l2h265enc ready (hevc;nv12;bgr8;bgr8)`.
Check with `ros2 topic hz /fcb/image/ffmpeg` (~59.9) and `ros2 topic echo /fcb/state`.

### Jetson specifics

* The GStreamer pipeline used is
  `appsrc (YUY2) → nvvidconv → NV12 (NVMM) → nvv4l2h265enc → h265parse → appsink`,
  with SPS/PPS inserted on every IDR; override it with `encoder.gst_pipeline`.
* Stock JetPack ffmpeg has no NVENC, so `backend: libav` means CPU encoding
  on the Orin. Keep `auto` or `gstreamer`.
* Viewing on the Orin: `view.launch.py` tries `hevc_cuvid` first, which does
  not exist on Jetson, and falls back to the software `hevc` decoder, which
  is fine for a preview. Viewing on the ground station over the network is
  preferable: run `view.launch.py` there.
* If the camera is wired to a Jetson UART instead of the board's USB serial:
  `visca_port:=/dev/ttyTHS1`.
* Run `sudo nvpmodel -m 0 && sudo jetson_clocks` for maximum performance.

## Testing without the camera

```bash
ros2 run fcb_camera fake_visca_camera.py             # prints a pty, e.g. /dev/pts/3
ros2 launch fcb_camera fcb_camera.launch.py visca_port:=/dev/pts/3 \
     video_device:=/dev/video1 width:=1280 height:=720 fps:=10.0   # any YUYV-capable webcam
colcon test --packages-select fcb_camera && colcon test-result --verbose
```

The fake camera implements ACK/completion semantics, all inquiries the
driver polls, zoom and focus travel time and error replies.

## Troubleshooting

| symptom | cause / fix |
|---|---|
| `camera not found: ... seen: /dev/video0 (...)` | board not enumerated or card name not matching; `lsusb | grep 04b4`, `cat /sys/class/video4linux/video*/name`; set `video.device` |
| `device refused pixel format` / `refused 1920x1080` | wrong node (metadata node, or the laptop webcam); use the index-0 NeoHD node |
| `no VISCA camera answered on: /dev/ttyACM0` | user not in `dialout`; another process holds the port (the driver takes an exclusive lock); wrong baud stored in the camera, try `visca.baud` |
| `no frames from /dev/video3 for 3 s` then `reopening in N s` | USB link hiccup. The driver backs off 5/10/20/40/60 s and, with the udev rule installed, resets the board over USB from the second attempt. `USB reset not possible: Permission denied` means the rule is missing. Without it, replug the camera. Check the cable and use a USB 3 port; `journalctl -k -f` shows `Non-zero status (-71)` or `connect-debounce failed` when the link itself is bad. |
| `encoder cannot keep up, dropping frames` | the encoder is slower than the camera. Use a hardware encoder (`hevc_nvenc`, `nvv4l2h265enc`), a faster preset, or `codec: h264`. |
| `frames_dropped` grows slowly with `capture_fps` < 59.9 | V4L2 sequence gaps, i.e. the USB link is losing frames; not an encoder problem |
| decoded image never appears in rqt | select the `ffmpeg` transport, and make sure `ffmpeg_image_transport` is installed on the viewing machine too |

## Design notes

* **Capture thread**: `poll` + `VIDIOC_DQBUF`, kernel monotonic timestamp
  converted to ROS time, one `memcpy` of the 4 MB frame into a pooled buffer,
  immediate requeue. Sequence gaps count as dropped frames.
* **Encode thread**: YUYV → yuv420p/NV12 (`swscale`, or `nvvidconv` on
  Jetson) → encoder. No B-frames, parameter sets repeated on every keyframe
  so a subscriber or a bag reader can start at any keyframe. Bounded queue:
  if the encoder falls behind, frames are dropped and counted instead of
  latency growing. Nothing on this path takes the VISCA lock; the zoom ratio
  used to scale `CameraInfo` is an atomic mirror of the polled state.
* **Timestamps**: packet pts is a frame counter; the node maps it back to the
  capture stamp when the packet comes out of the encoder, so encoders with
  internal delay still publish the true capture time.
* **VISCA link**: exclusive `flock` on the port, ACK/completion framing, late
  completions skipped, three consecutive poll failures drop the link and the
  state timer reconnects every 5 s. Services and the state poll run in their
  own callback group with a two-thread executor so a `wait: true` zoom never
  stalls publishing.
* **Recovery**: the detected device path is cached; a stalled stream is closed
  after 15 s of silence, then reopened with exponential backoff, with a
  `USBDEVFS_RESET` of the board from the second failure on. Progress is
  visible in `state.video_status`.
* **Packet format**: `encoding` is `codec;av_pixel_format;bgr8;bgr8`, the
  token string `ffmpeg_image_transport`'s subscriber uses to select a decoder
  and output `bgr8`, so any node using that transport decodes the stream
  without configuration.

## Verification status

Verified on the real camera on 2026-09-09: autodetection of both video and
VISCA, 1080p59.94 YUYV capture, H.265 encoding with `hevc_nvenc`, all
services listed above (including defog, noise reduction, presets and raw
inquiries), day/night switching visible in decoded frames, decoding through
`ffmpeg_image_transport`, bag recording, MP4 export and clean shutdown.
Board USB hiccups and the driver's backoff/reset recovery were exercised as
well. See the "test results" section at the end of this file for the numbers
of the last run.

Known gaps:

* `night_color` ICR sub-command has not been exercised on the camera.
* The calibration file is a placeholder from the datasheet field of view.
* The `night` picture is monochrome by design of the camera (IR cut filter
  removed); `night_color` keeps colour if the camera firmware supports it.

## Test results (2026-09-09, laptop, GTX 1650 Ti, `hevc_nvenc`)

Real camera on the Twiga USB3 NeoHD board, `ros2 launch fcb_camera fcb_camera.launch.py` with defaults.

| measurement | value |
|---|---|
| capture / publish rate right after start | 59.99 / 59.99 fps |
| encoded bitrate | 8.0 Mbit/s (cap), ~17 KB per packet |
| node CPU | 13 % of one core (colour conversion + NVENC upload) |
| zoom 5x→25x→1x with `wait: true` while streaming | publish rate stayed at 58.9 fps |
| 30 s bag | 16.6 MiB, i.e. ~35 MiB/min, 964 packets |
| `frames_dropped` | 126 in the first 35 s, all V4L2 sequence gaps (USB link), none from the encoder |
| services | all pass; `preset`, `defog`, `noise_reduction`, raw `visca` included |
| shutdown | clean in under 6 s, every run |

What did not pass is the USB link, not the driver: within a couple of
minutes of every run the kernel logged `uvcvideo: Non-zero status (-71)`,
the delivered frame rate sagged (59.9 → 46 fps with `capture_fps` and
`publish_fps` falling together) and then the board stopped delivering
frames altogether. The driver's recovery then ran as designed (backoff,
three `USBDEVFS_RESET`s, reopen) but this board does not come back from a
reset; only a replug restores it. Each USB reset also re-enumerates the
serial port, so the VISCA link drops and reconnects about 5 s later.

Before flying, test the same board on the Orin with a short, good USB 3
cable; if the -71 errors persist there, the board or cable is at fault.
