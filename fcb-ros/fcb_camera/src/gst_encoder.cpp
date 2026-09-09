// GStreamer backend: appsrc (YUY2) -> [converter] -> encoder -> parser -> appsink.
// On a Jetson the default pipeline uses nvvidconv + nvv4l2h265enc/nvv4l2h264enc,
// which is the only route to the Tegra hardware encoder from stock JetPack.
#include "fcb_camera/encoder.hpp"

#ifdef FCB_HAVE_GSTREAMER

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace fcb_camera
{
namespace
{
void ensureGstInit()
{
  static std::once_flag once;
  std::call_once(once, []() {
    if (!gst_is_initialized()) gst_init(nullptr, nullptr);
  });
}

bool haveElement(const char * name)
{
  ensureGstInit();
  GstElementFactory * f = gst_element_factory_find(name);
  if (!f) return false;
  gst_object_unref(f);
  return true;
}

std::string replaceAll(std::string s, const std::string & from, const std::string & to)
{
  size_t pos = 0;
  while ((pos = s.find(from, pos)) != std::string::npos) {
    s.replace(pos, from.size(), to);
    pos += to.size();
  }
  return s;
}

class GstEncoder : public Encoder
{
public:
  ~GstEncoder() override { close(); }

  void open(const EncoderConfig & cfg) override
  {
    close();
    ensureGstInit();
    cfg_ = cfg;
    codecName_ = cfg.codec == "h264" ? "h264" : "hevc";
    const std::string pipeline = buildPipeline(cfg);
    GError * err = nullptr;
    pipeline_ = gst_parse_launch(pipeline.c_str(), &err);
    if (!pipeline_ || err) {
      const std::string msg = err ? err->message : "unknown error";
      if (err) g_error_free(err);
      close();
      throw std::runtime_error("cannot build GStreamer pipeline: " + msg + " [" + pipeline + "]");
    }
    src_ = gst_bin_get_by_name(GST_BIN(pipeline_), "fcbsrc");
    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "fcbsink");
    if (!src_ || !sink_) {
      close();
      throw std::runtime_error("GStreamer pipeline must contain appsrc name=fcbsrc and appsink name=fcbsink");
    }
    frameDurationNs_ = static_cast<GstClockTime>(std::llround(1e9 / std::max(1.0, cfg.fps)));
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      close();
      throw std::runtime_error("GStreamer pipeline refused to start: " + pipeline);
    }
    pipelineDesc_ = pipeline;
    running_ = true;
    pullThread_ = std::thread([this]() { pullLoop(); });
    // Encoding string: the hardware path converts to NV12 before encoding.
    encoding_ = codecName_ + ";nv12;bgr8;bgr8";
  }

  void encode(const uint8_t * yuyv, int64_t pts, const PacketCallback & cb) override
  {
    checkBus();
    const size_t bytes = static_cast<size_t>(cfg_.stride) * cfg_.height;
    GstBuffer * buf = gst_buffer_new_allocate(nullptr, bytes, nullptr);
    GstMapInfo map;
    gst_buffer_map(buf, &map, GST_MAP_WRITE);
    std::memcpy(map.data, yuyv, bytes);
    gst_buffer_unmap(buf, &map);
    if (!haveFirstInput_) {
      firstInputPts_ = pts;
      haveFirstInput_ = true;
    }
    GST_BUFFER_PTS(buf) = static_cast<GstClockTime>(pts) * frameDurationNs_;
    GST_BUFFER_DTS(buf) = GST_BUFFER_PTS(buf);
    GST_BUFFER_DURATION(buf) = frameDurationNs_;
    const GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(src_), buf);  // takes ownership
    if (ret != GST_FLOW_OK) throw std::runtime_error("appsrc rejected buffer: " + std::string(gst_flow_get_name(ret)));
    deliver(cb);
  }

  void flush(const PacketCallback & cb) override
  {
    if (!pipeline_) return;
    gst_app_src_end_of_stream(GST_APP_SRC(src_));
    // wait briefly for the encoder to drain
    GstBus * bus = gst_element_get_bus(pipeline_);
    GstMessage * msg = gst_bus_timed_pop_filtered(bus, 2 * GST_SECOND,
                                                  static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
    if (msg) gst_message_unref(msg);
    gst_object_unref(bus);
    deliver(cb);
  }

  std::string encodingString() const override { return encoding_; }
  std::string name() const override { return "gstreamer:" + elementName_; }

private:
  std::string buildPipeline(const EncoderConfig & cfg)
  {
    const bool h264 = cfg.codec == "h264";
    std::string tmpl = cfg.gst_pipeline;
    std::string element = cfg.encoder_name;
    if (tmpl.empty()) {
      if (element.empty()) {
        if (haveElement(h264 ? "nvv4l2h264enc" : "nvv4l2h265enc")) element = h264 ? "nvv4l2h264enc" : "nvv4l2h265enc";
        else if (haveElement(h264 ? "x264enc" : "x265enc")) element = h264 ? "x264enc" : "x265enc";
        else throw std::runtime_error("no GStreamer encoder element found for " + cfg.codec);
      }
      const std::string parse = h264 ? "h264parse" : "h265parse";
      const std::string capsOut = h264 ? "video/x-h264,stream-format=byte-stream,alignment=au"
                                       : "video/x-h265,stream-format=byte-stream,alignment=au";
      tmpl = "appsrc name=fcbsrc is-live=true format=time do-timestamp=false block=false "
             "caps=video/x-raw,format=YUY2,width={width},height={height},framerate={fps_num}/{fps_den} "
             "! queue max-size-buffers=8 ";
      if (element.rfind("nvv4l2", 0) == 0) {
        tmpl += "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 "
                "! " + element + " bitrate={bitrate} peak-bitrate={peak_bitrate} control-rate=1 "
                "iframeinterval={gop} idrinterval={gop} insert-sps-pps=true insert-vui=true "
                "maxperf-enable=true preset-level=1 EnableTwopassCBR=false ";
      } else if (element == "x265enc") {
        tmpl += "! videoconvert ! video/x-raw,format=I420 ! x265enc bitrate={bitrate_kbps} key-int-max={gop} "
                "speed-preset=superfast tune=zerolatency option-string=repeat-headers=1 ";
      } else if (element == "x264enc") {
        tmpl += "! videoconvert ! video/x-raw,format=I420 ! x264enc bitrate={bitrate_kbps} key-int-max={gop} "
                "speed-preset=veryfast tune=zerolatency byte-stream=true ";
      } else {
        tmpl += "! videoconvert ! " + element + " ";
      }
      tmpl += "! " + parse + " config-interval=-1 ! " + capsOut +
              " ! appsink name=fcbsink sync=false max-buffers=16 drop=false emit-signals=false";
    }
    elementName_ = element.empty() ? "custom" : element;
    int fpsNum = static_cast<int>(std::lround(cfg.fps)), fpsDen = 1;
    if (std::fabs(cfg.fps - 59.94) < 0.02) { fpsNum = 60000; fpsDen = 1001; }
    tmpl = replaceAll(tmpl, "{width}", std::to_string(cfg.width));
    tmpl = replaceAll(tmpl, "{height}", std::to_string(cfg.height));
    tmpl = replaceAll(tmpl, "{fps_num}", std::to_string(fpsNum));
    tmpl = replaceAll(tmpl, "{fps_den}", std::to_string(fpsDen));
    tmpl = replaceAll(tmpl, "{fps}", std::to_string(static_cast<int>(std::lround(cfg.fps))));
    tmpl = replaceAll(tmpl, "{bitrate}", std::to_string(cfg.bit_rate));
    tmpl = replaceAll(tmpl, "{peak_bitrate}", std::to_string(cfg.bit_rate * 3 / 2));
    tmpl = replaceAll(tmpl, "{bitrate_kbps}", std::to_string(cfg.bit_rate / 1000));
    tmpl = replaceAll(tmpl, "{gop}", std::to_string(cfg.gop_size));
    tmpl = replaceAll(tmpl, "{quality}", std::to_string(cfg.quality));
    return tmpl;
  }

  void pullLoop()
  {
    while (running_) {
      GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), 100 * GST_MSECOND);
      if (!sample) {
        if (gst_app_sink_is_eos(GST_APP_SINK(sink_))) break;
        continue;
      }
      GstBuffer * buf = gst_sample_get_buffer(sample);
      if (buf) {
        Out out;
        out.keyframe = !GST_BUFFER_FLAG_IS_SET(buf, GST_BUFFER_FLAG_DELTA_UNIT);
        out.pts = frameIndexFor(buf);
        GstMapInfo map;
        if (gst_buffer_map(buf, &map, GST_MAP_READ)) {
          out.data.assign(map.data, map.data + map.size);
          gst_buffer_unmap(buf, &map);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        ready_.push_back(std::move(out));
      }
      gst_sample_unref(sample);
    }
  }

  // Encoders shift timestamps by a constant (GstVideoEncoder adds 1000 h so DTS
  // never goes negative), so the frame index is recovered relative to the first
  // packet, which always belongs to the first frame pushed.
  int64_t frameIndexFor(GstBuffer * buf)
  {
    const bool valid = GST_BUFFER_PTS_IS_VALID(buf) || GST_BUFFER_DTS_IS_VALID(buf);
    if (!valid || frameDurationNs_ == 0) return outputCount_++;
    const GstClockTime t = GST_BUFFER_PTS_IS_VALID(buf) ? GST_BUFFER_PTS(buf) : GST_BUFFER_DTS(buf);
    if (!haveOffset_) {
      ptsOffset_ = static_cast<int64_t>(t) - firstInputPts_ * static_cast<int64_t>(frameDurationNs_);
      haveOffset_ = true;
    }
    outputCount_++;
    const int64_t rel = static_cast<int64_t>(t) - ptsOffset_;
    return (rel + static_cast<int64_t>(frameDurationNs_) / 2) / static_cast<int64_t>(frameDurationNs_);
  }

  void deliver(const PacketCallback & cb)
  {
    std::deque<Out> batch;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      batch.swap(ready_);
    }
    for (auto & o : batch) {
      EncodedPacket p;
      p.pts = o.pts;
      p.keyframe = o.keyframe;
      p.data = o.data.data();
      p.size = o.data.size();
      cb(p);
    }
  }

  void checkBus()
  {
    GstBus * bus = gst_element_get_bus(pipeline_);
    GstMessage * msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
    gst_object_unref(bus);
    if (!msg) return;
    GError * err = nullptr;
    gchar * dbg = nullptr;
    gst_message_parse_error(msg, &err, &dbg);
    std::string text = err ? err->message : "unknown";
    if (dbg) { text += " (" + std::string(dbg) + ")"; g_free(dbg); }
    if (err) g_error_free(err);
    gst_message_unref(msg);
    throw std::runtime_error("GStreamer pipeline error: " + text);
  }

  void close()
  {
    running_ = false;
    if (pullThread_.joinable()) pullThread_.join();
    if (pipeline_) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (src_) { gst_object_unref(src_); src_ = nullptr; }
    if (sink_) { gst_object_unref(sink_); sink_ = nullptr; }
    if (pipeline_) { gst_object_unref(pipeline_); pipeline_ = nullptr; }
    std::lock_guard<std::mutex> lock(mutex_);
    ready_.clear();
    haveFirstInput_ = haveOffset_ = false;
    firstInputPts_ = ptsOffset_ = outputCount_ = 0;
  }

  struct Out
  {
    int64_t pts{0};
    bool keyframe{false};
    std::vector<uint8_t> data;
  };

  EncoderConfig cfg_;
  GstElement * pipeline_{nullptr};
  GstElement * src_{nullptr};
  GstElement * sink_{nullptr};
  GstClockTime frameDurationNs_{0};
  bool haveFirstInput_{false}, haveOffset_{false};
  int64_t firstInputPts_{0}, ptsOffset_{0}, outputCount_{0};
  std::thread pullThread_;
  std::atomic<bool> running_{false};
  std::mutex mutex_;
  std::deque<Out> ready_;
  std::string codecName_, elementName_, encoding_, pipelineDesc_;
};
}  // namespace

std::unique_ptr<Encoder> makeGstEncoder() { return std::make_unique<GstEncoder>(); }
bool gstreamerAvailable()
{
  return haveElement("nvv4l2h265enc") || haveElement("nvv4l2h264enc");
}
}  // namespace fcb_camera

#else  // !FCB_HAVE_GSTREAMER

namespace fcb_camera
{
std::unique_ptr<Encoder> makeGstEncoder() { return nullptr; }
bool gstreamerAvailable() { return false; }
}  // namespace fcb_camera

#endif
