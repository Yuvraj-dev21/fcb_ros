// Encoder backends. Input is always packed YUYV 4:2:2 straight from V4L2;
// output is an Annex-B elementary stream packet per frame, with in-band
// parameter sets on every keyframe so a subscriber can join mid-stream.
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace fcb_camera
{
struct EncoderConfig
{
  std::string backend{"auto"};    // "auto" | "libav" | "gstreamer"
  std::string codec{"hevc"};      // "hevc" | "h264"
  std::string encoder_name;       // libav encoder or GStreamer element; "" = pick automatically
  uint32_t width{1920};
  uint32_t height{1080};
  uint32_t stride{3840};          // bytes per input line (YUYV)
  double fps{60.0};
  int64_t bit_rate{8000000};      // bits/s cap
  int gop_size{30};
  int max_b_frames{0};
  int quality{23};                // crf / cq style constant-quality target; -1 = pure bitrate
  std::string preset;             // "" = backend default
  std::string av_options;         // "key:value,key:value" passed to the libav encoder
  std::string gst_pipeline;       // full GStreamer pipeline override with {placeholders}
  int threads{0};                 // libav: 0 = auto
};

struct EncodedPacket
{
  int64_t pts{0};        // frame index as handed to encode()
  bool keyframe{false};
  const uint8_t * data{nullptr};
  size_t size{0};
};

using PacketCallback = std::function<void(const EncodedPacket &)>;

class Encoder
{
public:
  virtual ~Encoder() = default;
  // Throws std::runtime_error when the backend cannot be opened.
  virtual void open(const EncoderConfig & cfg) = 0;
  // Encode one YUYV frame. The callback may be invoked zero or more times,
  // synchronously, from this thread.
  virtual void encode(const uint8_t * yuyv, int64_t pts, const PacketCallback & cb) = 0;
  virtual void flush(const PacketCallback & cb) = 0;
  // "hevc;yuv420p;bgr8;bgr8" style token string understood by ffmpeg_image_transport.
  virtual std::string encodingString() const = 0;
  // Human readable, e.g. "libav:hevc_nvenc".
  virtual std::string name() const = 0;
};

std::unique_ptr<Encoder> makeLibavEncoder();
std::unique_ptr<Encoder> makeGstEncoder();  // returns nullptr when compiled without GStreamer
bool gstreamerAvailable();
// Choose and open a backend per cfg.backend. `log` collects what was tried.
std::unique_ptr<Encoder> openEncoder(const EncoderConfig & cfg, std::string * log);
}  // namespace fcb_camera
