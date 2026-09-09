// Minimal V4L2 memory-mapped capture: dequeue, hand the buffer to the caller,
// requeue. No colour conversion, no copies, kernel timestamps preserved.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace fcb_camera
{
struct CaptureConfig
{
  std::string device;
  uint32_t width{1920};
  uint32_t height{1080};
  std::string fourcc{"YUYV"};
  double fps{60.0};
  uint32_t buffers{6};
};

struct CapturedFrame
{
  uint32_t index{0};        // V4L2 buffer index, pass back to requeue()
  const uint8_t * data{nullptr};
  size_t bytes{0};
  uint32_t sequence{0};
  int64_t timestamp_ns{0};  // CLOCK_MONOTONIC (or as flagged by the driver)
  bool monotonic{true};
  bool error{false};        // V4L2_BUF_FLAG_ERROR was set
};

struct DeviceInfo
{
  std::string path;
  std::string card;
  std::string driver;
  std::string bus_info;
  bool capture{false};
  std::vector<std::string> formats;
};

class V4l2Capture
{
public:
  V4l2Capture() = default;
  ~V4l2Capture();
  V4l2Capture(const V4l2Capture &) = delete;
  V4l2Capture & operator=(const V4l2Capture &) = delete;

  // Opens the device and negotiates format and frame rate. Throws std::runtime_error.
  void open(const CaptureConfig & cfg);
  // Allocates buffers and starts the stream. Throws std::runtime_error.
  void startStreaming();
  // open() + startStreaming().
  void start(const CaptureConfig & cfg);
  void stop();
  bool running() const { return fd_ >= 0 && streaming_; }

  // Wait up to timeout_ms for a frame. Returns false on timeout. Throws on I/O error.
  bool dequeue(CapturedFrame * frame, int timeout_ms);
  void requeue(uint32_t index);

  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }
  uint32_t stride() const { return stride_; }
  double fps() const { return fps_; }
  const std::string & card() const { return card_; }
  const std::string & device() const { return device_; }

  static DeviceInfo query(const std::string & path);
  // Issue USBDEVFS_RESET on the USB device behind a /dev/video* node (needs write
  // access to /dev/bus/usb/BBB/DDD, see the udev rule). Returns an error text or "".
  static std::string usbReset(const std::string & video_path);
  static std::vector<DeviceInfo> listDevices();
  // First /dev/video* whose card name contains one of `hints` (case-insensitive) and
  // that supports video capture in `fourcc`. Empty string when nothing matches.
  static std::string autodetect(const std::vector<std::string> & hints, const std::string & fourcc,
                                std::string * log);

private:
  struct Buffer
  {
    void * start{nullptr};
    size_t length{0};
  };
  int fd_{-1};
  bool streaming_{false};
  std::vector<Buffer> buffers_;
  uint32_t requestedBuffers_{6};
  uint32_t width_{0}, height_{0}, stride_{0};
  double fps_{0};
  std::string card_, device_;
};
}  // namespace fcb_camera
