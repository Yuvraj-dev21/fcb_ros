#include "fcb_camera/fcb_camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>

#include <unistd.h>

#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace fcb_camera
{
namespace
{
int64_t monotonicNowNs()
{
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}
}  // namespace

FcbCameraNode::FcbCameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fcb_camera", options), lastStatsTime_(0, 0, RCL_ROS_TIME), lastViscaAttempt_(0, 0, RCL_ROS_TIME)
{
  declareParameters();

  const int depth = std::max(10, 2 * encCfg_.gop_size);
  auto qos = rclcpp::QoS(rclcpp::KeepLast(depth)).reliable();
  packetPub_ = create_publisher<FFMPEGPacket>("image/ffmpeg", qos);
  infoPub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
  statePub_ = create_publisher<CameraState>("state", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());

  infoManager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, cameraName_, cameraInfoUrl_);
  if (!infoManager_->isCalibrated()) {
    RCLCPP_WARN(get_logger(), "no camera calibration loaded (camera_info_url='%s'); CameraInfo will carry zeros",
                cameraInfoUrl_.c_str());
  }

  serviceGroup_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  zoomSrv_ = create_service<fcb_interfaces::srv::SetZoom>(
    "set_zoom", std::bind(&FcbCameraNode::onSetZoom, this, _1, _2), rmw_qos_profile_services_default, serviceGroup_);
  focusSrv_ = create_service<fcb_interfaces::srv::SetFocus>(
    "set_focus", std::bind(&FcbCameraNode::onSetFocus, this, _1, _2), rmw_qos_profile_services_default, serviceGroup_);
  icrSrv_ = create_service<fcb_interfaces::srv::SetIcr>(
    "set_icr", std::bind(&FcbCameraNode::onSetIcr, this, _1, _2), rmw_qos_profile_services_default, serviceGroup_);
  exposureSrv_ = create_service<fcb_interfaces::srv::SetExposure>(
    "set_exposure", std::bind(&FcbCameraNode::onSetExposure, this, _1, _2), rmw_qos_profile_services_default,
    serviceGroup_);
  wbSrv_ = create_service<fcb_interfaces::srv::SetWhiteBalance>(
    "set_white_balance", std::bind(&FcbCameraNode::onSetWhiteBalance, this, _1, _2),
    rmw_qos_profile_services_default, serviceGroup_);
  pictureSrv_ = create_service<fcb_interfaces::srv::SetPicture>(
    "set_picture", std::bind(&FcbCameraNode::onSetPicture, this, _1, _2), rmw_qos_profile_services_default,
    serviceGroup_);
  presetSrv_ = create_service<fcb_interfaces::srv::Preset>(
    "preset", std::bind(&FcbCameraNode::onPreset, this, _1, _2), rmw_qos_profile_services_default, serviceGroup_);
  viscaSrv_ = create_service<fcb_interfaces::srv::ViscaCommand>(
    "visca", std::bind(&FcbCameraNode::onVisca, this, _1, _2), rmw_qos_profile_services_default, serviceGroup_);
  reconnectSrv_ = create_service<std_srvs::srv::Trigger>(
    "reconnect_visca", std::bind(&FcbCameraNode::onReconnect, this, _1, _2), rmw_qos_profile_services_default,
    serviceGroup_);

  state_.zoom_ratio = 1.0f;
  state_.shutter = state_.iris = state_.gain = -1;
  state_.focus_mode = state_.icr_mode = state_.exposure_mode = state_.white_balance_mode = "unknown";

  {
    std::lock_guard<std::mutex> lock(viscaMutex_);
    connectVisca();
    if (visca_) applyStartupSettings();
  }
  if (!visca_ && viscaRequired_) {
    throw std::runtime_error("VISCA control link not found and visca.required is true");
  }

  const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, stateRateHz_));
  stateTimer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                  std::bind(&FcbCameraNode::stateTimer, this), serviceGroup_);

  startVideo();
}

FcbCameraNode::~FcbCameraNode()
{
  stopVideo();
  std::lock_guard<std::mutex> lock(viscaMutex_);
  visca_.reset();
}

// ---------------------------------------------------------------------------
// parameters
// ---------------------------------------------------------------------------

void FcbCameraNode::declareParameters()
{
  videoDevice_ = declare_parameter<std::string>("video.device", "");
  deviceHints_ = declare_parameter<std::vector<std::string>>("video.device_hints", {"NeoHD", "FCB", "Harrier"});
  width_ = declare_parameter<int>("video.width", 1920);
  height_ = declare_parameter<int>("video.height", 1080);
  fps_ = declare_parameter<double>("video.fps", 60.0);
  videoFourcc_ = declare_parameter<std::string>("video.pixel_format", "YUYV");
  v4l2Buffers_ = declare_parameter<int>("video.buffers", 6);
  timestampSource_ = declare_parameter<std::string>("video.timestamp_source", "v4l2");

  frameId_ = declare_parameter<std::string>("frame_id", "fcb_optical_frame");
  cameraName_ = declare_parameter<std::string>("camera_name", "fcb_ev9520l");
  cameraInfoUrl_ = declare_parameter<std::string>("camera_info_url", "");
  scaleInfoWithZoom_ = declare_parameter<bool>("camera_info.scale_with_zoom", true);

  encCfg_.backend = declare_parameter<std::string>("encoder.backend", "auto");
  encCfg_.codec = lower(declare_parameter<std::string>("encoder.codec", "hevc"));
  if (encCfg_.codec == "h265") encCfg_.codec = "hevc";
  encCfg_.encoder_name = declare_parameter<std::string>("encoder.name", "");
  encCfg_.bit_rate = declare_parameter<int64_t>("encoder.bit_rate", 8000000);
  encCfg_.gop_size = declare_parameter<int>("encoder.gop_size", 30);
  encCfg_.max_b_frames = declare_parameter<int>("encoder.max_b_frames", 0);
  encCfg_.quality = declare_parameter<int>("encoder.quality", 23);
  encCfg_.preset = declare_parameter<std::string>("encoder.preset", "");
  encCfg_.av_options = declare_parameter<std::string>("encoder.av_options", "");
  encCfg_.gst_pipeline = declare_parameter<std::string>("encoder.gst_pipeline", "");
  encCfg_.threads = declare_parameter<int>("encoder.threads", 0);
  queueDepth_ = std::max(1, static_cast<int>(declare_parameter<int>("encoder.queue_depth", 4)));

  viscaPort_ = declare_parameter<std::string>("visca.port", "");
  viscaBaud_ = declare_parameter<int>("visca.baud", 0);
  viscaAddress_ = declare_parameter<int>("visca.address", 1);
  viscaTimeoutS_ = declare_parameter<double>("visca.timeout", 0.5);
  viscaRequired_ = declare_parameter<bool>("visca.required", false);
  allowDigitalZoom_ = declare_parameter<bool>("visca.allow_digital_zoom", false);

  stateRateHz_ = declare_parameter<double>("state_rate_hz", 2.0);

  declare_parameter<std::string>("startup.icr_mode", "");
  declare_parameter<std::string>("startup.focus_mode", "");
  declare_parameter<std::string>("startup.exposure_mode", "");
  declare_parameter<std::string>("startup.stabilizer", "");
  declare_parameter<std::string>("startup.digital_zoom", "");
  declare_parameter<double>("startup.zoom_ratio", -1.0);
}

// ---------------------------------------------------------------------------
// video pipeline
// ---------------------------------------------------------------------------

void FcbCameraNode::startVideo()
{
  running_ = true;
  encodeThread_ = std::thread([this]() { encodeLoop(); });
  captureThread_ = std::thread([this]() { captureLoop(); });
}

void FcbCameraNode::stopVideo()
{
  running_ = false;
  queueCv_.notify_all();
  if (captureThread_.joinable()) captureThread_.join();
  if (encodeThread_.joinable()) encodeThread_.join();
  std::lock_guard<std::mutex> lock(encoderMutex_);
  if (encoder_) {
    try {
      encoder_->flush([this](const EncodedPacket & p) { onPacket(p); });
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "encoder flush failed: %s", e.what());
    }
    encoder_.reset();
  }
  capture_.stop();
}

bool FcbCameraNode::openVideo()
{
  std::string device = videoDevice_;
  if (device.empty()) {
    // Reuse the node found last time as long as it still exists: every
    // autodetect sweep opens each /dev/video* once, and this board dislikes churn.
    if (!detectedDevice_.empty() && ::access(detectedDevice_.c_str(), F_OK) == 0) {
      device = detectedDevice_;
    } else {
      std::string log;
      device = V4l2Capture::autodetect(deviceHints_, videoFourcc_, &log);
      if (device.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
                             "camera not found: %s -- is it plugged in? (set video.device to force a node)",
                             log.c_str());
        setVideoStatus("no device");
        return false;
      }
      RCLCPP_INFO(get_logger(), "video autodetect: %s", log.c_str());
      detectedDevice_ = device;
    }
  }
  CaptureConfig cfg;
  cfg.device = device;
  cfg.width = static_cast<uint32_t>(width_);
  cfg.height = static_cast<uint32_t>(height_);
  cfg.fourcc = videoFourcc_;
  cfg.fps = fps_;
  cfg.buffers = static_cast<uint32_t>(std::max(2, v4l2Buffers_));
  try {
    capture_.open(cfg);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "cannot configure %s: %s", device.c_str(), e.what());
    return false;
  }
  // Open the encoder before the first buffer is queued so no frames pile up
  // in the kernel while a hardware encoder initialises.
  if (!openEncoderFor(capture_.width(), capture_.height(), capture_.stride(), capture_.fps())) {
    capture_.stop();
    return false;
  }
  try {
    capture_.startStreaming();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "cannot start streaming on %s: %s", device.c_str(),
                          e.what());
    capture_.stop();
    return false;
  }
  RCLCPP_INFO(get_logger(), "streaming %ux%u %s @ %.2f fps from %s (\"%s\")", capture_.width(), capture_.height(),
              videoFourcc_.c_str(), capture_.fps(), device.c_str(), capture_.card().c_str());
  haveSequence_ = false;
  setVideoStatus("streaming");
  {
    std::lock_guard<std::mutex> lock(viscaMutex_);
    state_.video_device = device;
    state_.width = capture_.width();
    state_.height = capture_.height();
  }
  return true;
}

bool FcbCameraNode::openEncoderFor(uint32_t width, uint32_t height, uint32_t stride, double fps)
{
  std::lock_guard<std::mutex> lock(encoderMutex_);
  if (encoder_ && encWidth_ == width && encHeight_ == height && encStride_ == stride && encFps_ == fps) {
    return true;
  }
  if (encoder_) {
    try {
      encoder_->flush([this](const EncodedPacket & p) { onPacket(p); });
    } catch (...) {
    }
    encoder_.reset();
  }
  EncoderConfig cfg = encCfg_;
  cfg.width = width;
  cfg.height = height;
  cfg.stride = stride;
  cfg.fps = fps;
  std::string log;
  encoder_ = openEncoder(cfg, &log);
  if (!encoder_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "cannot open any encoder: %s", log.c_str());
    return false;
  }
  encoding_ = encoder_->encodingString();
  encWidth_ = width;
  encHeight_ = height;
  encStride_ = stride;
  encFps_ = fps;
  RCLCPP_INFO(get_logger(), "encoder %s ready (%s), %.1f Mbit/s cap, gop %d, quality %d [%s]",
              encoder_->name().c_str(), encoding_.c_str(), cfg.bit_rate / 1e6, cfg.gop_size, cfg.quality,
              log.c_str());
  std::lock_guard<std::mutex> vlock(viscaMutex_);
  state_.encoder = encoder_->name();
  return true;
}

rclcpp::Time FcbCameraNode::stampFor(const CapturedFrame & f)
{
  const rclcpp::Time now = this->now();
  if (timestampSource_ == "v4l2" && f.monotonic && f.timestamp_ns > 0) {
    const int64_t age = monotonicNowNs() - f.timestamp_ns;
    if (age >= 0 && age < 2000000000LL) {
      return now - rclcpp::Duration(std::chrono::nanoseconds(age));
    }
  }
  return now;
}

void FcbCameraNode::setVideoStatus(const std::string & status)
{
  std::lock_guard<std::mutex> lock(viscaMutex_);
  state_.video_status = status;
}

// Sleep in small steps so shutdown stays responsive.
void FcbCameraNode::backoffSleep(double seconds)
{
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (running_ && rclcpp::ok() && std::chrono::steady_clock::now() < until) {
    std::this_thread::sleep_for(200ms);
  }
}

void FcbCameraNode::captureLoop()
{
  int idleMs = 0;
  int failures = 0;       // consecutive open failures or stalls
  bool gotFramesSinceOpen = false;
  while (running_ && rclcpp::ok()) {
    if (!capture_.running()) {
      streaming_ = false;
      if (failures > 0) {
        // 5, 10, 20, 40, 60, 60 ... seconds between attempts. A USB reset of the
        // board is tried from the second failure on; on this board a stalled
        // stream never comes back from a plain reopen.
        const double wait = std::min(60.0, 5.0 * std::pow(2.0, failures - 1));
        if (failures >= 2 && !detectedDevice_.empty()) {
          const std::string err = V4l2Capture::usbReset(detectedDevice_);
          if (err.empty()) {
            RCLCPP_WARN(get_logger(), "issued a USB reset to the camera board");
            setVideoStatus("usb reset");
            backoffSleep(3.0);
          } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000, "USB reset not possible: %s", err.c_str());
          }
        }
        char buf[96];
        std::snprintf(buf, sizeof(buf), "reopening in %.0f s (attempt %d)", wait, failures + 1);
        setVideoStatus(buf);
        RCLCPP_WARN(get_logger(), "video: %s", buf);
        backoffSleep(wait);
        if (!running_) break;
      }
      if (!openVideo()) {
        failures++;
        continue;
      }
      idleMs = 0;
      gotFramesSinceOpen = false;
    }
    CapturedFrame f;
    bool got = false;
    try {
      got = capture_.dequeue(&f, 500);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "capture error: %s", e.what());
      capture_.stop();
      streaming_ = false;
      failures++;
      continue;
    }
    if (!got) {
      idleMs += 500;
      if (idleMs == 3000) {
        RCLCPP_WARN(get_logger(), "no frames from %s for 3 s", capture_.device().c_str());
        setVideoStatus("stalled");
      }
      if (idleMs >= 15000) {
        // The NeoHD board is known to wedge after open/close churn, so reopen only
        // after a long silence rather than at the first hiccup.
        RCLCPP_ERROR(get_logger(), "no frames for 15 s, closing the video device");
        capture_.stop();
        streaming_ = false;
        failures++;
      }
      continue;
    }
    idleMs = 0;
    streaming_ = true;
    if (!gotFramesSinceOpen) {
      gotFramesSinceOpen = true;
      failures = 0;
      setVideoStatus("streaming");
    }
    if (haveSequence_ && f.sequence > lastSequence_ + 1) {
      framesDropped_ += (f.sequence - lastSequence_ - 1);
    }
    lastSequence_ = f.sequence;
    haveSequence_ = true;
    if (f.error) {
      capture_.requeue(f.index);
      framesDropped_++;
      continue;
    }
    framesCaptured_++;
    const size_t need = static_cast<size_t>(capture_.stride()) * capture_.height();
    if (f.bytes < need) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "short frame: %zu bytes, expected %zu", f.bytes, need);
      capture_.requeue(f.index);
      framesDropped_++;
      continue;
    }
    const rclcpp::Time stamp = stampFor(f);

    std::unique_ptr<Frame> frame;
    {
      std::lock_guard<std::mutex> lock(queueMutex_);
      if (queue_.size() >= static_cast<size_t>(queueDepth_)) {
        framesDropped_++;
        capture_.requeue(f.index);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "encoder cannot keep up, dropping frames (queue depth %d)", queueDepth_);
        continue;
      }
      if (!framePool_.empty()) {
        frame = std::move(framePool_.back());
        framePool_.pop_back();
      }
    }
    if (!frame) frame = std::make_unique<Frame>();
    frame->yuyv.resize(need);
    std::memcpy(frame->yuyv.data(), f.data, need);
    capture_.requeue(f.index);
    frame->stamp = stamp;
    frame->pts = nextPts_++;
    {
      std::lock_guard<std::mutex> lock(ptsMutex_);
      ptsToStamp_[frame->pts] = stamp;
      if (ptsToStamp_.size() > 512) {
        // an encoder that never returned some pts; drop the oldest to bound memory
        auto oldest = std::min_element(ptsToStamp_.begin(), ptsToStamp_.end(),
                                       [](auto & a, auto & b) { return a.first < b.first; });
        ptsToStamp_.erase(oldest);
      }
    }
    {
      std::lock_guard<std::mutex> lock(queueMutex_);
      queue_.push_back(std::move(frame));
    }
    queueCv_.notify_one();
  }
  capture_.stop();
  streaming_ = false;
}

void FcbCameraNode::encodeLoop()
{
  const PacketCallback cb = [this](const EncodedPacket & p) { onPacket(p); };
  while (running_) {
    std::unique_ptr<Frame> frame;
    {
      std::unique_lock<std::mutex> lock(queueMutex_);
      queueCv_.wait(lock, [this]() { return !running_ || !queue_.empty(); });
      if (!running_) break;
      frame = std::move(queue_.front());
      queue_.pop_front();
    }
    {
      std::lock_guard<std::mutex> lock(encoderMutex_);
      if (encoder_) {
        try {
          encoder_->encode(frame->yuyv.data(), frame->pts, cb);
        } catch (const std::exception & e) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "encode failed: %s", e.what());
          framesDropped_++;
        }
      }
    }
    std::lock_guard<std::mutex> lock(queueMutex_);
    if (framePool_.size() < static_cast<size_t>(queueDepth_) + 2) framePool_.push_back(std::move(frame));
  }
}

void FcbCameraNode::onPacket(const EncodedPacket & pkt)
{
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock(ptsMutex_);
    auto it = ptsToStamp_.find(pkt.pts);
    if (it == ptsToStamp_.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "packet pts %ld has no capture stamp", pkt.pts);
      stamp = this->now();
    } else {
      stamp = it->second;
      ptsToStamp_.erase(it);
    }
  }
  auto msg = std::make_unique<FFMPEGPacket>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frameId_;
  msg->encoding = encoding_;
  msg->width = static_cast<int32_t>(encWidth_);
  msg->height = static_cast<int32_t>(encHeight_);
  msg->pts = static_cast<uint64_t>(pkt.pts);
  msg->flags = pkt.keyframe ? 1 : 0;
  msg->is_bigendian = false;
  msg->data.assign(pkt.data, pkt.data + pkt.size);
  bytesPublished_ += pkt.size;
  packetsPublished_++;
  packetPub_->publish(std::move(msg));

  auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(infoManager_->getCameraInfo());
  info->header.stamp = stamp;
  info->header.frame_id = frameId_;
  if (info->width == 0 || info->height == 0) {
    info->width = encWidth_;
    info->height = encHeight_;
  }
  if (scaleInfoWithZoom_) {
    // Lock-free: viscaMutex_ is held for hundreds of ms during serial polls,
    // and this runs once per frame on the encode thread.
    const float ratio = zoomRatio_.load(std::memory_order_relaxed);
    if (ratio > 1.0f && info->k[0] > 0.0) {
      info->k[0] *= ratio;
      info->k[4] *= ratio;
      if (info->p.size() == 12) {
        info->p[0] *= ratio;
        info->p[5] *= ratio;
      }
    }
  }
  infoPub_->publish(std::move(info));
}

// ---------------------------------------------------------------------------
// VISCA link and state
// ---------------------------------------------------------------------------

void FcbCameraNode::connectVisca()
{
  // caller holds viscaMutex_
  lastViscaAttempt_ = this->now();
  const auto timeout = Ms(static_cast<int>(viscaTimeoutS_ * 1000));
  std::vector<int> bauds = viscaBaud_ > 0 ? std::vector<int>{viscaBaud_} : std::vector<int>{9600, 38400, 115200};
  std::vector<std::string> ports = viscaPort_.empty() ? ViscaLink::defaultPortCandidates()
                                                      : std::vector<std::string>{viscaPort_};
  const auto [port, baud] = ViscaLink::autodetect(ports, bauds, static_cast<uint8_t>(viscaAddress_), timeout);
  if (port.empty()) {
    std::string tried;
    for (auto & p : ports) tried += p + " ";
    RCLCPP_WARN(get_logger(), "no VISCA camera answered on: %s(bauds %s) -- control services unavailable",
                tried.empty() ? "(no serial ports) " : tried.c_str(), viscaBaud_ > 0 ? std::to_string(viscaBaud_).c_str() : "9600/38400/115200");
    state_.control_connected = false;
    return;
  }
  try {
    visca_ = std::make_unique<ViscaLink>(port, baud, static_cast<uint8_t>(viscaAddress_), timeout);
    try {
      visca_->commandNoAck(visca::ifClear(viscaAddress_));
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(get_logger(), "IF_Clear: %s", e.what());
    }
    try {
      const auto v = visca_->inquiry(visca::versionInq(viscaAddress_));
      if (v.size() >= 7) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "0x%02x%02x", v[3], v[4]);
        state_.model_id = buf;
        std::snprintf(buf, sizeof(buf), "0x%02x%02x", v[5], v[6]);
        state_.rom_version = buf;
      }
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(get_logger(), "version inquiry: %s", e.what());
    }
    state_.control_connected = true;
    state_.visca_port = port;
    state_.visca_baud = baud;
    if (!allowDigitalZoom_) {
      // Otherwise a continuous "tele" rolls straight past 30x into digital zoom.
      try {
        visca_->command(visca::digitalZoom(viscaAddress_, false));
        state_.digital_zoom_enabled = false;
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "could not disable digital zoom: %s", e.what());
      }
    }
    RCLCPP_INFO(get_logger(), "VISCA camera on %s @ %d baud (model %s, rom %s)", port.c_str(), baud,
                state_.model_id.c_str(), state_.rom_version.c_str());
    pollCamera(true);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "VISCA open failed on %s: %s", port.c_str(), e.what());
    visca_.reset();
    state_.control_connected = false;
  }
}

void FcbCameraNode::applyStartupSettings()
{
  // caller holds viscaMutex_ and visca_ is valid
  const uint8_t a = static_cast<uint8_t>(viscaAddress_);
  auto onoff = [&](const std::string & param, auto && builder, const char * what) {
    const std::string v = lower(get_parameter(param).as_string());
    if (v.empty()) return;
    bool on = false;
    if (!parseOnOff(v, &on)) {
      RCLCPP_WARN(get_logger(), "startup.%s: expected on/off, got '%s'", what, v.c_str());
      return;
    }
    try {
      visca_->command(builder(a, on));
      RCLCPP_INFO(get_logger(), "startup: %s %s", what, on ? "on" : "off");
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "startup: %s failed: %s", what, e.what());
    }
  };
  onoff("startup.stabilizer", [](uint8_t ad, bool on) { return visca::stabilizer(ad, on); }, "stabilizer");
  onoff("startup.digital_zoom", [](uint8_t ad, bool on) { return visca::digitalZoom(ad, on); }, "digital_zoom");

  const std::string icr = visca::normalizeIcrMode(get_parameter("startup.icr_mode").as_string());
  if (!get_parameter("startup.icr_mode").as_string().empty()) {
    auto req = std::make_shared<fcb_interfaces::srv::SetIcr::Request>();
    auto res = std::make_shared<fcb_interfaces::srv::SetIcr::Response>();
    req->mode = icr.empty() ? get_parameter("startup.icr_mode").as_string() : icr;
    // onSetIcr locks viscaMutex_; run the body without re-locking
    viscaMutex_.unlock();
    onSetIcr(req, res);
    viscaMutex_.lock();
    RCLCPP_INFO(get_logger(), "startup: icr %s -> %s", req->mode.c_str(), res->message.c_str());
  }
  const std::string focus = lower(get_parameter("startup.focus_mode").as_string());
  if (!focus.empty()) {
    auto req = std::make_shared<fcb_interfaces::srv::SetFocus::Request>();
    auto res = std::make_shared<fcb_interfaces::srv::SetFocus::Response>();
    req->mode = focus;
    viscaMutex_.unlock();
    onSetFocus(req, res);
    viscaMutex_.lock();
    RCLCPP_INFO(get_logger(), "startup: focus %s -> %s", focus.c_str(), res->message.c_str());
  }
  const std::string ae = lower(get_parameter("startup.exposure_mode").as_string());
  if (!ae.empty()) {
    auto req = std::make_shared<fcb_interfaces::srv::SetExposure::Request>();
    auto res = std::make_shared<fcb_interfaces::srv::SetExposure::Response>();
    req->mode = ae;
    viscaMutex_.unlock();
    onSetExposure(req, res);
    viscaMutex_.lock();
    RCLCPP_INFO(get_logger(), "startup: exposure %s -> %s", ae.c_str(), res->message.c_str());
  }
  const double zoom = get_parameter("startup.zoom_ratio").as_double();
  if (zoom >= 1.0) {
    auto req = std::make_shared<fcb_interfaces::srv::SetZoom::Request>();
    auto res = std::make_shared<fcb_interfaces::srv::SetZoom::Response>();
    req->mode = "ratio";
    req->value = zoom;
    viscaMutex_.unlock();
    onSetZoom(req, res);
    viscaMutex_.lock();
    RCLCPP_INFO(get_logger(), "startup: zoom %.1fx -> %s", zoom, res->message.c_str());
  }
}

void FcbCameraNode::pollCamera(bool everything)
{
  // caller holds viscaMutex_ and visca_ is valid
  const uint8_t a = static_cast<uint8_t>(viscaAddress_);
  auto nib4 = [&](const visca::Packet & p) { return visca::parseNibbles(visca_->inquiry(p), 4); };
  auto byte = [&](const visca::Packet & p) { return visca::parseByte(visca_->inquiry(p)); };

  const int zoom = nib4(visca::zoomPosInq(a));
  if (zoom >= 0) {
    state_.zoom_position = zoom;
    state_.zoom_ratio = static_cast<float>(visca::positionToRatio(zoom));
    zoomRatio_.store(state_.zoom_ratio, std::memory_order_relaxed);
  }
  const int focus = nib4(visca::focusPosInq(a));
  if (focus >= 0) state_.focus_position = focus;
  if (!everything) return;

  int v;
  if ((v = byte(visca::focusModeInq(a))) >= 0) state_.focus_mode = v == 0x02 ? "auto" : v == 0x03 ? "manual" : "unknown";
  if ((v = byte(visca::digitalZoomInq(a))) >= 0) state_.digital_zoom_enabled = (v == 0x02);
  if ((v = byte(visca::icrInq(a))) >= 0) state_.icr_mode = visca::icrModeName(v);
  if ((v = byte(visca::autoIcrInq(a))) >= 0) state_.auto_icr = (v == 0x02 || v == 0x04);
  if ((v = byte(visca::aeModeInq(a))) >= 0) state_.exposure_mode = visca::aeModeName(v);
  if ((v = nib4(visca::shutterInq(a))) >= 0) state_.shutter = v;
  if ((v = nib4(visca::irisInq(a))) >= 0) state_.iris = v;
  if ((v = nib4(visca::gainInq(a))) >= 0) state_.gain = v;
  if ((v = byte(visca::wbModeInq(a))) >= 0) state_.white_balance_mode = visca::wbModeName(v);
  if ((v = byte(visca::stabilizerInq(a))) >= 0) state_.stabilizer_on = (v == 0x02);
  if ((v = byte(visca::powerInq(a))) >= 0) state_.power_on = (v == 0x02);
}

void FcbCameraNode::stateTimer()
{
  // stream statistics
  const rclcpp::Time now = this->now();
  const uint64_t cap = framesCaptured_, pub = packetsPublished_, bytes = bytesPublished_;
  if (lastStatsTime_.nanoseconds() == 0) {
    lastStatsTime_ = now;
    lastCaptured_ = cap;
    lastPublished_ = pub;
    lastBytes_ = bytes;
  }
  const double dt = (now - lastStatsTime_).seconds();
  if (dt > 0.2) {
    captureFps_ = static_cast<float>((cap - lastCaptured_) / dt);
    publishFps_ = static_cast<float>((pub - lastPublished_) / dt);
    bitrateMbps_ = static_cast<float>((bytes - lastBytes_) * 8.0 / dt / 1e6);
    lastCaptured_ = cap;
    lastPublished_ = pub;
    lastBytes_ = bytes;
    lastStatsTime_ = now;
  }

  std::lock_guard<std::mutex> lock(viscaMutex_);
  if (!visca_) {
    if ((now - lastViscaAttempt_).seconds() > 5.0) connectVisca();
  } else {
    try {
      slowPollCounter_ = (slowPollCounter_ + 1) % std::max(1, static_cast<int>(std::lround(stateRateHz_ * 2)));
      pollCamera(slowPollCounter_ == 0);
      viscaFailures_ = 0;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "VISCA poll failed: %s", e.what());
      // Three failed polls in a row means the link is gone; drop it and let the timer reconnect.
      if (++viscaFailures_ >= 3) {
        RCLCPP_ERROR(get_logger(), "VISCA link lost on %s; will retry every 5 s", visca_->port().c_str());
        visca_.reset();
        state_.control_connected = false;
        viscaFailures_ = 0;
      }
    }
  }
  publishState();
}

void FcbCameraNode::publishState()
{
  // caller holds viscaMutex_
  state_.header.stamp = this->now();
  state_.header.frame_id = frameId_;
  state_.streaming = streaming_;
  if (!state_.streaming && state_.video_status == "streaming") state_.video_status = "stalled";
  state_.capture_fps = captureFps_;
  state_.publish_fps = publishFps_;
  state_.bitrate_mbps = bitrateMbps_;
  state_.frames_captured = framesCaptured_;
  state_.frames_dropped = framesDropped_;
  statePub_->publish(state_);
}

// ---------------------------------------------------------------------------
// service helpers
// ---------------------------------------------------------------------------

bool FcbCameraNode::parseOnOff(const std::string & s, bool * value)
{
  const std::string v = lower(s);
  if (v == "on" || v == "true" || v == "1" || v == "yes") { *value = true; return true; }
  if (v == "off" || v == "false" || v == "0" || v == "no") { *value = false; return true; }
  return false;
}

bool FcbCameraNode::requireVisca(std::string * message)
{
  if (visca_) return true;
  *message = "VISCA control link is not connected";
  return false;
}

template <typename Fn>
bool FcbCameraNode::withVisca(Fn && fn, std::string * message)
{
  if (!requireVisca(message)) return false;
  try {
    fn();
    return true;
  } catch (const ViscaError & e) {
    *message = std::string("camera refused: ") + e.what();
  } catch (const ViscaTimeout & e) {
    *message = std::string("camera did not answer: ") + e.what();
  } catch (const std::exception & e) {
    *message = e.what();
  }
  return false;
}

// ---------------------------------------------------------------------------
// services
// ---------------------------------------------------------------------------

void FcbCameraNode::onSetZoom(const std::shared_ptr<fcb_interfaces::srv::SetZoom::Request> req,
                              std::shared_ptr<fcb_interfaces::srv::SetZoom::Response> res)
{
  std::lock_guard<std::mutex> lock(viscaMutex_);
  const uint8_t a = static_cast<uint8_t>(viscaAddress_);
  const std::string mode = lower(req->mode);
  const bool digital = allowDigitalZoom_ || state_.digital_zoom_enabled;
  res->success = withVisca([&]() {
    if (mode == "ratio" || mode == "position") {
      int target = mode == "ratio" ? visca::ratioToPosition(req->value) : static_cast<int>(std::lround(req->value));
      const int clamped = visca::clampZoom(target, digital);
      if (clamped != target) {
        res->message = "requested zoom beyond " + std::string(digital ? "digital" : "optical") +
                       " range, clamped; ";
        target = clamped;
      }
      visca_->command(visca::zoomDirect(a, target), req->wait, Ms(10000));
    } else if (mode == "tele") {
      visca_->command(visca::zoomTele(a, req->speed));
    } else if (mode == "wide") {
      visca_->command(visca::zoomWide(a, req->speed));
    } else if (mode == "stop") {
      visca_->command(visca::zoomStop(a));
    } else if (mode == "digital") {
      visca_->command(visca::digitalZoom(a, req->value >= 1.0));
      state_.digital_zoom_enabled = req->value >= 1.0;
    } else {
      throw std::runtime_error("unknown zoom mode '" + req->mode + "' (ratio|position|tele|wide|stop|digital)");
    }
    const int pos = visca::parseNibbles(visca_->inquiry(visca::zoomPosInq(a)), 4);
    if (pos >= 0) {
      state_.zoom_position = pos;
      state_.zoom_ratio = static_cast<float>(visca::positionToRatio(pos));
      zoomRatio_.store(state_.zoom_ratio, std::memory_order_relaxed);
    }
    res->message += "ok";
  }, &res->message);
  res->zoom_position = state_.zoom_position;
  res->zoom_ratio = state_.zoom_ratio;
}

void FcbCameraNode::onSetFocus(const std::shared_ptr<fcb_interfaces::srv::SetFocus::Request> req,
                               std::shared_ptr<fcb_interfaces::srv::SetFocus::Response> res)
{
  std::lock_guard<std::mutex> lock(viscaMutex_);
  const uint8_t a = static_cast<uint8_t>(viscaAddress_);
  const std::string mode = lower(req->mode);
  res->success = withVisca([&]() {
    if (mode == "auto") {
      visca_->command(visca::focusMode(a, true));
      state_.focus_mode = "auto";
    } else if (mode == "manual") {
      visca_->command(visca::focusMode(a, false));
      state_.focus_mode = "manual";
    } else if (mode == "one_push" || mode == "onepush" || mode == "one-push") {
      visca_->command(visca::focusMode(a, false));
      visca_->command(visca::focusOnePush(a), true, Ms(5000));
      state_.focus_mode = "manual";
    } else if (mode == "position") {
      visca_->command(visca::focusMode(a, false));
      visca_->command(visca::focusDirect(a, req->position), true, Ms(5000));
      state_.focus_mode = "manual";
    } else if (mode == "far") {
      visca_->command(visca::focusMode(a, false));
      visca_->command(visca::focusFar(a, req->speed));
      state_.focus_mode = "manual";
    } else if (mode == "near") {
      visca_->command(visca::focusMode(a, false));
      visca_->command(visca::focusNear(a, req->speed));
      state_.focus_mode = "manual";
    } else if (mode == "stop") {
      visca_->command(visca::focusStop(a));
    } else {
      throw std::runtime_error("unknown focus mode '" + req->mode + "' (auto|manual|one_push|position|far|near|stop)");
    }
    const int pos = visca::parseNibbles(visca_->inquiry(visca::focusPosInq(a)), 4);
    if (pos >= 0) state_.focus_position = pos;
    res->message = "ok";
  }, &res->message);
  res->focus_position = state_.focus_position;
  res->focus_mode = state_.focus_mode;
}

void FcbCameraNode::onSetIcr(const std::shared_ptr<fcb_interfaces::srv::SetIcr::Request> req,
                             std::shared_ptr<fcb_interfaces::srv::SetIcr::Response> res)
{
  std::lock_guard<std::mutex> lock(viscaMutex_);
  const uint8_t a = static_cast<uint8_t>(viscaAddress_);
  const std::string mode = visca::normalizeIcrMode(req->mode);
  res->success = withVisca([&]() {
    if (mode.empty()) {
      throw std::runtime_error("unknown ICR mode '" + req->mode +
                               "' (day|night|night_color|auto|auto_color|manual)");
    }
    if (mode == "auto" || mode == "auto_color") {
      visca_->command(visca::autoIcr(a, mode == "auto" ? 0x02 : 0x04));
      state_.auto_icr = true;
    } else if (mode == "manual") {
      visca_->command(visca::autoIcr(a, 0x03));
      state_.auto_icr = false;
    } else {
      // manual selection requires auto switching off first
      visca_->command(visca::autoIcr(a, 0x03));
      state_.auto_icr = false;
      const uint8_t sub = mode == "day" ? 0x03 : mode == "night" ? 0x02 : 0x04;
      visca_->command(visca::icr(a, sub), true, Ms(3000));
    }
    if (req->auto_threshold >= 0) visca_->command(visca::autoIcrThreshold(a, req->auto_threshold));
    const int v = visca::parseByte(visca_->inquiry(visca::icrInq(a)));
    if (v >= 0) state_.icr_mode = visca::icrModeName(v);
    res->message = "ok";
  }, &res->message);
  res->icr_mode = state_.icr_mode;
  res->auto_icr = state_.auto_icr;
}

void FcbCameraNode::onSetExposure(const std::shared_ptr<fcb_interfaces::srv::SetExposure::Request> req,
                                  std::shared_ptr<fcb_interfaces::srv::SetExposure::Response> res)
{
  std::lock_guard<std::mutex> lock(viscaMutex_);
  const uint8_t a = static_cast<uint8_t>(viscaAddress_);
  res->success = withVisca([&]() {
    if (!req->mode.empty()) {
      const int code = visca::aeModeCode(req->mode);
      if (code < 0) {
        throw std::runtime_error("unknown exposure mode '" + req->mode +
                                 "' (full_auto|manual|shutter_priority|iris_priority|bright)");
      }
      visca_->command(visca::aeMode(a, static_cast<uint8_t>(code)));
      state_.exposure_mode = visca::aeModeName(code);
    }
    if (req->shutter >= 0) visca_->command(visca::shutterDirect(a, req->shutter));
    if (req->iris >= 0) visca_->command(visca::irisDirect(a, req->iris));
    if (req->gain >= 0) visca_->command(visca::gainDirect(a, req->gain));
    if (req->gain_limit >= 0) visca_->command(visca::gainLimit(a, req->gain_limit));
    if (req->brightness >= 0) visca_->command(visca::brightDirect(a, req->brightness));
    bool on;
    if (!req->exposure_compensation_enable.empty()) {
      if (!parseOnOff(req->exposure_compensation_enable, &on)) throw std::runtime_error("exposure_compensation_enable: on|off");
      visca_->command(visca::expCompOnOff(a, on));
    }
    if (req->exposure_compensation >= 0) {
      visca_->command(visca::expCompOnOff(a, true));
      visca_->command(visca::expCompDirect(a, req->exposure_compensation));
    }
    if (!req->backlight_compensation.empty()) {
      if (!parseOnOff(req->backlight_compensation, &on)) throw std::runtime_error("backlight_compensation: on|off");
      visca_->command(visca::backlight(a, on));
    }
    if (!req->slow_shutter_auto.empty()) {
      if (!parseOnOff(req->slow_shutter_auto, &on)) throw std::runtime_error("slow_shutter_auto: on|off");
      visca_->command(visca::slowShutterAuto(a, on));
    }
    int v;
    if ((v = visca::parseNibbles(visca_->inquiry(visca::shutterInq(a)), 4)) >= 0) state_.shutter = v;
    if ((v = visca::parseNibbles(visca_->inquiry(visca::irisInq(a)), 4)) >= 0) state_.iris = v;
    if ((v = visca::parseNibbles(visca_->inquiry(visca::gainInq(a)), 4)) >= 0) state_.gain = v;
    res->message = "ok";
  }, &res->message);
  res->exposure_mode = state_.exposure_mode;
  res->shutter = state_.shutter;
  res->iris = state_.iris;
  res->gain = state_.gain;
}

void FcbCameraNode::onSetWhiteBalance(const std::shared_ptr<fcb_interfaces::srv::SetWhiteBalance::Request> req,
                                      std::shared_ptr<fcb_interfaces::srv::SetWhiteBalance::Response> res)
{
  std::lock_guard<std::mutex> lock(viscaMutex_);
  const uint8_t a = static_cast<uint8_t>(viscaAddress_);
  res->success = withVisca([&]() {
    if (!req->mode.empty()) {
      const int code = visca::wbModeCode(req->mode);
      if (code < 0) throw std::runtime_error("unknown white balance mode '" + req->mode + "'");
      visca_->command(visca::wbMode(a, static_cast<uint8_t>(code)));
      state_.white_balance_mode = visca::wbModeName(code);
    }
    if (req->trigger_one_push) {
      if (state_.white_balance_mode != "one_push") {
        visca_->command(visca::wbMode(a, 0x03));
        state_.white_balance_mode = "one_push";
      }
      visca_->command(visca::wbOnePushTrigger(a), true, Ms(5000));
    }
    if (req->red_gain >= 0) visca_->command(visca::rGainDirect(a, req->red_gain));
    if (req->blue_gain >= 0) visca_->command(visca::bGainDirect(a, req->blue_gain));
    const int v = visca::parseByte(visca_->inquiry(visca::wbModeInq(a)));
    if (v >= 0) state_.white_balance_mode = visca::wbModeName(v);
    res->message = "ok";
  }, &res->message);
  res->white_balance_mode = state_.white_balance_mode;
}

void FcbCameraNode::onSetPicture(const std::shared_ptr<fcb_interfaces::srv::SetPicture::Request> req,
                                 std::shared_ptr<fcb_interfaces::srv::SetPicture::Response> res)
{
  std::lock_guard<std::mutex> lock(viscaMutex_);
  const uint8_t a = static_cast<uint8_t>(viscaAddress_);
  res->success = withVisca([&]() {
    bool on;
    auto flag = [&](const std::string & field, const char * name) -> int {
      if (field.empty()) return -1;
      if (!parseOnOff(field, &on)) throw std::runtime_error(std::string(name) + ": expected on|off");
      return on ? 1 : 0;
    };
    int v;
    if ((v = flag(req->stabilizer, "stabilizer")) >= 0) {
      visca_->command(visca::stabilizer(a, v));
      state_.stabilizer_on = v;
    }
    if ((v = flag(req->defog, "defog")) >= 0) visca_->command(visca::defog(a, v, req->defog_level > 0 ? req->defog_level : 1));
    if (req->noise_reduction >= 0) visca_->command(visca::noiseReduction(a, req->noise_reduction));
    if (req->aperture >= 0) visca_->command(visca::apertureDirect(a, req->aperture));
    if ((v = flag(req->flip, "flip")) >= 0) visca_->command(visca::pictureFlip(a, v));
    if ((v = flag(req->mirror, "mirror")) >= 0) visca_->command(visca::mirror(a, v));
    if ((v = flag(req->freeze, "freeze")) >= 0) visca_->command(visca::freeze(a, v));
    if ((v = flag(req->digital_zoom, "digital_zoom")) >= 0) {
      visca_->command(visca::digitalZoom(a, v));
      state_.digital_zoom_enabled = v;
    }
    if ((v = flag(req->power, "power")) >= 0) {
      visca_->command(visca::power(a, v), false, Ms(5000));
      state_.power_on = v;
    }
    res->message = "ok";
  }, &res->message);
}

void FcbCameraNode::onPreset(const std::shared_ptr<fcb_interfaces::srv::Preset::Request> req,
                             std::shared_ptr<fcb_interfaces::srv::Preset::Response> res)
{
  std::lock_guard<std::mutex> lock(viscaMutex_);
  const uint8_t a = static_cast<uint8_t>(viscaAddress_);
  const std::string action = lower(req->action);
  res->success = withVisca([&]() {
    if (req->slot > 15) throw std::runtime_error("slot must be 0..15");
    uint8_t code;
    if (action == "set" || action == "store") code = 0x01;
    else if (action == "recall") code = 0x02;
    else if (action == "reset" || action == "clear") code = 0x00;
    else throw std::runtime_error("unknown preset action '" + req->action + "' (set|recall|reset)");
    visca_->command(visca::memory(a, code, req->slot), code == 0x02, Ms(10000));
    if (code == 0x02) pollCamera(true);
    res->message = "ok";
  }, &res->message);
}

void FcbCameraNode::onVisca(const std::shared_ptr<fcb_interfaces::srv::ViscaCommand::Request> req,
                            std::shared_ptr<fcb_interfaces::srv::ViscaCommand::Response> res)
{
  std::lock_guard<std::mutex> lock(viscaMutex_);
  const uint8_t a = static_cast<uint8_t>(viscaAddress_);
  res->success = withVisca([&]() {
    visca::Packet body(req->payload.begin(), req->payload.end());
    if (body.empty() && !visca::parseHex(req->hex, &body)) throw std::runtime_error("hex string is malformed");
    if (body.empty()) throw std::runtime_error("empty VISCA payload");
    // Accept a payload that already carries the header and/or terminator.
    if ((body.front() & 0xF8) == 0x80 || body.front() == 0x88) body.erase(body.begin());
    if (!body.empty() && body.back() == 0xFF) body.pop_back();
    const Ms timeout(static_cast<int>(std::max(0.05f, req->timeout_s) * 1000));
    const visca::Packet frame = visca::wrap(a, body);
    if (req->inquiry) {
      const auto reply = visca_->inquiry(frame, timeout);
      res->reply.assign(reply.begin(), reply.end());
      res->reply_hex = visca::hex(reply);
    } else {
      visca_->command(frame, req->wait_completion, timeout);
    }
    res->message = "ok";
  }, &res->message);
}

void FcbCameraNode::onReconnect(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  std::lock_guard<std::mutex> lock(viscaMutex_);
  visca_.reset();
  state_.control_connected = false;
  connectVisca();
  res->success = static_cast<bool>(visca_);
  res->message = visca_ ? "connected on " + visca_->port() : "no VISCA camera found";
}

}  // namespace fcb_camera

RCLCPP_COMPONENTS_REGISTER_NODE(fcb_camera::FcbCameraNode)
