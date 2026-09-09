#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "fcb_camera/encoder.hpp"
#include "fcb_camera/v4l2_capture.hpp"
#include "fcb_camera/visca_link.hpp"
#include "fcb_interfaces/msg/camera_state.hpp"
#include "fcb_interfaces/srv/preset.hpp"
#include "fcb_interfaces/srv/set_exposure.hpp"
#include "fcb_interfaces/srv/set_focus.hpp"
#include "fcb_interfaces/srv/set_icr.hpp"
#include "fcb_interfaces/srv/set_picture.hpp"
#include "fcb_interfaces/srv/set_white_balance.hpp"
#include "fcb_interfaces/srv/set_zoom.hpp"
#include "fcb_interfaces/srv/visca_command.hpp"

namespace fcb_camera
{
class FcbCameraNode : public rclcpp::Node
{
public:
  explicit FcbCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~FcbCameraNode() override;

private:
  using FFMPEGPacket = ffmpeg_image_transport_msgs::msg::FFMPEGPacket;
  using CameraState = fcb_interfaces::msg::CameraState;
  using Ms = std::chrono::milliseconds;

  // A captured frame copied out of the V4L2 buffer, waiting for the encoder.
  struct Frame
  {
    std::vector<uint8_t> yuyv;
    rclcpp::Time stamp;
    int64_t pts{0};
  };

  // ---- setup
  void declareParameters();
  void startVideo();
  void stopVideo();
  bool openVideo();
  bool openEncoderFor(uint32_t width, uint32_t height, uint32_t stride, double fps);
  void connectVisca();
  void applyStartupSettings();

  // ---- pipeline threads
  void captureLoop();
  void setVideoStatus(const std::string & status);
  void backoffSleep(double seconds);
  void encodeLoop();
  void onPacket(const EncodedPacket & pkt);
  rclcpp::Time stampFor(const CapturedFrame & f);

  // ---- state / diagnostics
  void stateTimer();
  void pollCamera(bool everything);
  void publishState();

  // ---- services
  void onSetZoom(const std::shared_ptr<fcb_interfaces::srv::SetZoom::Request> req,
                 std::shared_ptr<fcb_interfaces::srv::SetZoom::Response> res);
  void onSetFocus(const std::shared_ptr<fcb_interfaces::srv::SetFocus::Request> req,
                  std::shared_ptr<fcb_interfaces::srv::SetFocus::Response> res);
  void onSetIcr(const std::shared_ptr<fcb_interfaces::srv::SetIcr::Request> req,
                std::shared_ptr<fcb_interfaces::srv::SetIcr::Response> res);
  void onSetExposure(const std::shared_ptr<fcb_interfaces::srv::SetExposure::Request> req,
                     std::shared_ptr<fcb_interfaces::srv::SetExposure::Response> res);
  void onSetWhiteBalance(const std::shared_ptr<fcb_interfaces::srv::SetWhiteBalance::Request> req,
                         std::shared_ptr<fcb_interfaces::srv::SetWhiteBalance::Response> res);
  void onSetPicture(const std::shared_ptr<fcb_interfaces::srv::SetPicture::Request> req,
                    std::shared_ptr<fcb_interfaces::srv::SetPicture::Response> res);
  void onPreset(const std::shared_ptr<fcb_interfaces::srv::Preset::Request> req,
                std::shared_ptr<fcb_interfaces::srv::Preset::Response> res);
  void onVisca(const std::shared_ptr<fcb_interfaces::srv::ViscaCommand::Request> req,
               std::shared_ptr<fcb_interfaces::srv::ViscaCommand::Response> res);
  void onReconnect(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Run `fn` against the VISCA link; catches link errors into (success, message).
  template <typename Fn>
  bool withVisca(Fn && fn, std::string * message);
  bool requireVisca(std::string * message);
  static bool parseOnOff(const std::string & s, bool * value);

  // ---- parameters
  std::string videoDevice_, videoFourcc_, frameId_, cameraName_, cameraInfoUrl_;
  std::string detectedDevice_;
  std::vector<std::string> deviceHints_;
  int width_{1920}, height_{1080}, v4l2Buffers_{6};
  double fps_{60.0};
  std::string timestampSource_{"v4l2"};
  bool scaleInfoWithZoom_{true};
  int queueDepth_{4};
  double stateRateHz_{2.0};
  bool allowDigitalZoom_{false};
  EncoderConfig encCfg_;
  std::string viscaPort_;
  int viscaBaud_{0};
  int viscaAddress_{1};
  double viscaTimeoutS_{0.5};
  bool viscaRequired_{false};

  // ---- video pipeline
  V4l2Capture capture_;
  std::unique_ptr<Encoder> encoder_;
  std::mutex encoderMutex_;
  std::string encoding_;
  uint32_t encWidth_{0}, encHeight_{0}, encStride_{0};
  double encFps_{0};
  std::thread captureThread_, encodeThread_;
  std::atomic<bool> running_{false};
  std::mutex queueMutex_;
  std::condition_variable queueCv_;
  std::deque<std::unique_ptr<Frame>> queue_;
  std::vector<std::unique_ptr<Frame>> framePool_;
  std::mutex ptsMutex_;
  std::unordered_map<int64_t, rclcpp::Time> ptsToStamp_;
  int64_t nextPts_{0};
  std::atomic<bool> streaming_{false};

  // ---- stats (written by pipeline threads, read by state timer)
  std::atomic<uint64_t> framesCaptured_{0}, framesDropped_{0}, packetsPublished_{0}, bytesPublished_{0};
  uint64_t lastCaptured_{0}, lastPublished_{0}, lastBytes_{0};
  rclcpp::Time lastStatsTime_;
  float captureFps_{0}, publishFps_{0}, bitrateMbps_{0};
  uint32_t lastSequence_{0};
  bool haveSequence_{false};

  // ---- VISCA
  std::unique_ptr<ViscaLink> visca_;
  std::mutex viscaMutex_;  // guards visca_ pointer and cached state
  CameraState state_;
  std::atomic<float> zoomRatio_{1.0f};  // mirror of state_.zoom_ratio readable without viscaMutex_
  rclcpp::Time lastViscaAttempt_;
  int slowPollCounter_{0};
  int viscaFailures_{0};

  // ---- ROS
  rclcpp::Publisher<FFMPEGPacket>::SharedPtr packetPub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub_;
  rclcpp::Publisher<CameraState>::SharedPtr statePub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> infoManager_;
  rclcpp::TimerBase::SharedPtr stateTimer_;
  rclcpp::CallbackGroup::SharedPtr serviceGroup_;
  rclcpp::Service<fcb_interfaces::srv::SetZoom>::SharedPtr zoomSrv_;
  rclcpp::Service<fcb_interfaces::srv::SetFocus>::SharedPtr focusSrv_;
  rclcpp::Service<fcb_interfaces::srv::SetIcr>::SharedPtr icrSrv_;
  rclcpp::Service<fcb_interfaces::srv::SetExposure>::SharedPtr exposureSrv_;
  rclcpp::Service<fcb_interfaces::srv::SetWhiteBalance>::SharedPtr wbSrv_;
  rclcpp::Service<fcb_interfaces::srv::SetPicture>::SharedPtr pictureSrv_;
  rclcpp::Service<fcb_interfaces::srv::Preset>::SharedPtr presetSrv_;
  rclcpp::Service<fcb_interfaces::srv::ViscaCommand>::SharedPtr viscaSrv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnectSrv_;
};
}  // namespace fcb_camera
