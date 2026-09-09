#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
#include <libswscale/swscale.h>
}

#include "fcb_camera/encoder.hpp"

namespace fcb_camera
{
namespace
{
std::string avErr(int err)
{
  char buf[AV_ERROR_MAX_STRING_SIZE] = {0};
  av_strerror(err, buf, sizeof(buf));
  return buf;
}

std::vector<std::pair<std::string, std::string>> splitOptions(const std::string & s)
{
  std::vector<std::pair<std::string, std::string>> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    const auto k = item.find(':');
    if (k == std::string::npos) {
      const auto e = item.find('=');
      if (e == std::string::npos) continue;
      out.emplace_back(item.substr(0, e), item.substr(e + 1));
    } else {
      out.emplace_back(item.substr(0, k), item.substr(k + 1));
    }
  }
  return out;
}

std::vector<std::string> candidateEncoders(const std::string & codec)
{
  if (codec == "h264") return {"h264_nvenc", "h264_v4l2m2m", "libx264"};
  return {"hevc_nvenc", "hevc_v4l2m2m", "libx265"};
}

class LibavEncoder : public Encoder
{
public:
  ~LibavEncoder() override { close(); }

  void open(const EncoderConfig & cfg) override
  {
    close();
    cfg_ = cfg;
    const AVCodec * codec = nullptr;
    if (!cfg.encoder_name.empty()) {
      codec = avcodec_find_encoder_by_name(cfg.encoder_name.c_str());
      if (!codec) throw std::runtime_error("libav encoder not found: " + cfg.encoder_name);
      encoderName_ = cfg.encoder_name;
    } else {
      for (const auto & name : candidateEncoders(cfg.codec)) {
        codec = avcodec_find_encoder_by_name(name.c_str());
        if (codec && tryOpen(codec, cfg, /*probe=*/true)) {
          encoderName_ = name;
          break;
        }
        codec = nullptr;
      }
      if (!codec) throw std::runtime_error("no usable libav encoder for codec " + cfg.codec);
    }
    if (codec->id == AV_CODEC_ID_HEVC) codecName_ = "hevc";
    else if (codec->id == AV_CODEC_ID_H264) codecName_ = "h264";
    else codecName_ = avcodec_get_name(codec->id);
    if (!tryOpen(codec, cfg, /*probe=*/false)) {
      throw std::runtime_error("cannot open libav encoder " + encoderName_ + ": " + lastError_);
    }
  }

  void encode(const uint8_t * yuyv, int64_t pts, const PacketCallback & cb) override
  {
    const uint8_t * src[4] = {yuyv, nullptr, nullptr, nullptr};
    const int srcStride[4] = {static_cast<int>(cfg_.stride), 0, 0, 0};
    sws_scale(sws_, src, srcStride, 0, static_cast<int>(cfg_.height), frame_->data, frame_->linesize);
    frame_->pts = pts;
    int ret = avcodec_send_frame(ctx_, frame_);
    if (ret < 0) throw std::runtime_error("avcodec_send_frame: " + avErr(ret));
    drain(cb);
  }

  void flush(const PacketCallback & cb) override
  {
    if (!ctx_) return;
    avcodec_send_frame(ctx_, nullptr);
    drain(cb);
  }

  std::string encodingString() const override { return encoding_; }
  std::string name() const override { return "libav:" + encoderName_; }

private:
  bool tryOpen(const AVCodec * codec, const EncoderConfig & cfg, bool probe)
  {
    closeCodec();
    ctx_ = avcodec_alloc_context3(codec);
    if (!ctx_) { lastError_ = "cannot allocate codec context"; return false; }
    ctx_->width = static_cast<int>(cfg.width);
    ctx_->height = static_cast<int>(cfg.height);
    // Integer frame rate for the time base; 59.94 becomes 60000/1001.
    int fpsNum = static_cast<int>(std::lround(cfg.fps)), fpsDen = 1;
    if (std::abs(cfg.fps - 59.94) < 0.02) { fpsNum = 60000; fpsDen = 1001; }
    ctx_->time_base = AVRational{fpsDen, fpsNum};
    ctx_->framerate = AVRational{fpsNum, fpsDen};
    ctx_->gop_size = cfg.gop_size;
    ctx_->max_b_frames = cfg.max_b_frames;
    ctx_->bit_rate = cfg.bit_rate;
    ctx_->rc_max_rate = cfg.bit_rate;
    ctx_->rc_buffer_size = static_cast<int>(std::min<int64_t>(cfg.bit_rate * 2, 1LL << 30));
    ctx_->thread_count = cfg.threads;
    ctx_->pix_fmt = pickPixelFormat(codec);
    if (ctx_->pix_fmt == AV_PIX_FMT_NONE) { lastError_ = "no supported pixel format"; return false; }
    // No GLOBAL_HEADER: parameter sets are repeated in-band on every keyframe.
    ctx_->flags &= ~AV_CODEC_FLAG_GLOBAL_HEADER;

    applyDefaults(codec->name, cfg);
    for (const auto & [k, v] : splitOptions(cfg.av_options)) setOpt(k, v, true);

    int ret = avcodec_open2(ctx_, codec, nullptr);
    if (ret < 0) { lastError_ = avErr(ret); closeCodec(); return false; }
    if (probe) { closeCodec(); return true; }

    frame_ = av_frame_alloc();
    frame_->format = ctx_->pix_fmt;
    frame_->width = ctx_->width;
    frame_->height = ctx_->height;
    ret = av_frame_get_buffer(frame_, 64);
    if (ret < 0) { lastError_ = avErr(ret); closeCodec(); return false; }
    packet_ = av_packet_alloc();
    sws_ = sws_getContext(ctx_->width, ctx_->height, AV_PIX_FMT_YUYV422, ctx_->width, ctx_->height,
                          ctx_->pix_fmt, SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws_) { lastError_ = "cannot create swscale context"; closeCodec(); return false; }
    encoding_ = codecName_ + ";" + av_get_pix_fmt_name(ctx_->pix_fmt) + ";bgr8;bgr8";
    return true;
  }

  AVPixelFormat pickPixelFormat(const AVCodec * codec)
  {
    if (!codec->pix_fmts) return AV_PIX_FMT_YUV420P;
    // Prefer formats swscale reaches cheaply from YUYV and that every decoder handles.
    for (AVPixelFormat want : {AV_PIX_FMT_YUV420P, AV_PIX_FMT_NV12, AV_PIX_FMT_YUV422P, AV_PIX_FMT_YUYV422}) {
      for (const AVPixelFormat * p = codec->pix_fmts; *p != AV_PIX_FMT_NONE; ++p) {
        if (*p == want) return want;
      }
    }
    const AVPixFmtDescriptor * d = av_pix_fmt_desc_get(codec->pix_fmts[0]);
    if (d && !(d->flags & AV_PIX_FMT_FLAG_HWACCEL)) return codec->pix_fmts[0];
    return AV_PIX_FMT_NONE;
  }

  void applyDefaults(const std::string & enc, const EncoderConfig & cfg)
  {
    const std::string q = std::to_string(cfg.quality);
    const bool isNvenc = enc.find("nvenc") != std::string::npos;
    const bool isX26x = enc == "libx264" || enc == "libx265";
    if (isX26x) {
      setOpt("preset", cfg.preset.empty() ? (enc == "libx265" ? "superfast" : "veryfast") : cfg.preset, false);
      setOpt("tune", "zerolatency", false);
      if (cfg.quality >= 0) setOpt("crf", q, false);
      if (enc == "libx265") {
        // keep VPS/SPS/PPS with every keyframe and avoid the x265 lookahead latency
        setOpt("x265-params", "repeat-headers=1:log-level=error", false);
      }
    } else if (isNvenc) {
      setOpt("preset", cfg.preset.empty() ? "p4" : cfg.preset, false);
      setOpt("tune", "ll", false);
      setOpt("rc", cfg.quality >= 0 ? "vbr" : "cbr", false);
      if (cfg.quality >= 0) setOpt("cq", q, false);
      // No "delay 0": that makes each encode call wait for the GPU, which halves
      // throughput at 1080p60. Packets come back a few frames later; the pts map
      // in the node re-attaches the capture timestamps.
      setOpt("zerolatency", "1", false);
    } else if (!cfg.preset.empty()) {
      setOpt("preset", cfg.preset, false);
    }
  }

  void setOpt(const std::string & k, const std::string & v, bool strict)
  {
    const int ret = av_opt_set(ctx_->priv_data, k.c_str(), v.c_str(), AV_OPT_SEARCH_CHILDREN);
    if (ret < 0) {
      const int ret2 = av_opt_set(ctx_, k.c_str(), v.c_str(), AV_OPT_SEARCH_CHILDREN);
      if (ret2 < 0 && strict) {
        throw std::runtime_error("libav option '" + k + "=" + v + "' rejected: " + avErr(ret2));
      }
    }
  }

  void drain(const PacketCallback & cb)
  {
    for (;;) {
      const int ret = avcodec_receive_packet(ctx_, packet_);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) return;
      if (ret < 0) throw std::runtime_error("avcodec_receive_packet: " + avErr(ret));
      EncodedPacket p;
      p.pts = packet_->pts;
      p.keyframe = (packet_->flags & AV_PKT_FLAG_KEY) != 0;
      p.data = packet_->data;
      p.size = static_cast<size_t>(packet_->size);
      cb(p);
      av_packet_unref(packet_);
    }
  }

  void closeCodec()
  {
    if (sws_) { sws_freeContext(sws_); sws_ = nullptr; }
    if (packet_) av_packet_free(&packet_);
    if (frame_) av_frame_free(&frame_);
    if (ctx_) avcodec_free_context(&ctx_);
  }
  void close() { closeCodec(); }

  EncoderConfig cfg_;
  AVCodecContext * ctx_{nullptr};
  AVFrame * frame_{nullptr};
  AVPacket * packet_{nullptr};
  SwsContext * sws_{nullptr};
  std::string encoderName_, codecName_, encoding_, lastError_;
};
}  // namespace

std::unique_ptr<Encoder> makeLibavEncoder() { return std::make_unique<LibavEncoder>(); }

std::unique_ptr<Encoder> openEncoder(const EncoderConfig & cfg, std::string * log)
{
  std::string notes;
  auto attempt = [&](const std::string & which) -> std::unique_ptr<Encoder> {
    std::unique_ptr<Encoder> enc = (which == "gstreamer") ? makeGstEncoder() : makeLibavEncoder();
    if (!enc) {
      notes += which + ": not compiled in; ";
      return nullptr;
    }
    try {
      enc->open(cfg);
      notes += which + ": opened " + enc->name() + "; ";
      return enc;
    } catch (const std::exception & e) {
      notes += which + ": " + e.what() + "; ";
      return nullptr;
    }
  };
  std::unique_ptr<Encoder> enc;
  if (cfg.backend == "libav") {
    enc = attempt("libav");
  } else if (cfg.backend == "gstreamer") {
    enc = attempt("gstreamer");
  } else {
    // auto: the Jetson hardware encoder is only reachable through GStreamer, so
    // try that first when its element exists, otherwise libav (NVENC or CPU).
    if (gstreamerAvailable()) enc = attempt("gstreamer");
    if (!enc) enc = attempt("libav");
  }
  if (log) *log = notes;
  return enc;
}
}  // namespace fcb_camera
