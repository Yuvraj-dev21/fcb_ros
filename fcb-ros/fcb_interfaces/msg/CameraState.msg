# Periodic snapshot of the Sony FCB-EV9520L: lens state, image settings and
# stream statistics. Published by fcb_camera on ~/state at `state_rate_hz`.
#
# Lens and image fields are read back from the camera over VISCA. When the
# control link is down (`control_connected` false) they hold the last known
# values, or the defaults below (-1 / "unknown").

std_msgs/Header header

# ---- control link ----------------------------------------------------------
bool control_connected          # VISCA link is open and the camera answers
string visca_port               # serial device in use, e.g. /dev/ttyACM0
int32 visca_baud
string model_id                 # from CAM_VersionInq, hex, e.g. "0x0a50"
string rom_version              # from CAM_VersionInq, hex

# ---- lens ------------------------------------------------------------------
int32 zoom_position             # raw VISCA zoom position, 0x0000 (wide) .. 0x4000 (30x optical) .. 0x7AC0 (digital)
float32 zoom_ratio              # magnification: 1.0 .. 30.0 optical, up to 360.0 with digital zoom
bool digital_zoom_enabled
int32 focus_position            # raw VISCA focus position
string focus_mode               # "auto" | "manual" | "unknown"

# ---- image -----------------------------------------------------------------
string icr_mode                 # IR cut filter: "day" | "night" | "night_color" | "unknown"
bool auto_icr                   # camera switches day/night by itself
string exposure_mode            # "full_auto" | "manual" | "shutter_priority" | "iris_priority" | "bright" | "unknown"
int32 shutter                   # raw shutter position index (-1 unknown)
int32 iris                      # raw iris position index (-1 unknown)
int32 gain                      # raw gain position index (-1 unknown)
string white_balance_mode       # "auto" | "indoor" | "outdoor" | "one_push" | "atw" | "manual" | "outdoor_auto" | "sodium_auto" | "sodium" | "sodium_outdoor_auto" | "unknown"
bool stabilizer_on
bool power_on

# ---- video stream ----------------------------------------------------------
bool streaming                  # V4L2 stream is open and delivering frames
string video_status             # "streaming" | "stalled" | "reopening in N s (attempt K)" | "no device" | "usb reset"
string video_device             # e.g. /dev/video0
uint32 width
uint32 height
float32 capture_fps             # frames dequeued from V4L2 over the last second
float32 publish_fps             # encoded packets published over the last second
float32 bitrate_mbps            # encoded output over the last second
uint64 frames_captured
uint64 frames_dropped           # V4L2 sequence gaps + frames dropped because the encoder fell behind
string encoder                  # e.g. "libav:hevc_nvenc" or "gstreamer:nvv4l2h265enc"
