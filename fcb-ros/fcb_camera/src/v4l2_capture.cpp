#include "fcb_camera/v4l2_capture.hpp"

#include <fcntl.h>
#include <glob.h>
#include <linux/usbdevice_fs.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <climits>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace fcb_camera
{
namespace
{
int xioctl(int fd, unsigned long req, void * arg)
{
  int r;
  do {
    r = ::ioctl(fd, req, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

uint32_t fourccCode(const std::string & s)
{
  if (s.size() != 4) throw std::runtime_error("pixel format must be a 4-character fourcc, got '" + s + "'");
  return v4l2_fourcc(s[0], s[1], s[2], s[3]);
}

std::string fourccString(uint32_t code)
{
  std::string s(4, ' ');
  for (int i = 0; i < 4; ++i) s[i] = static_cast<char>((code >> (8 * i)) & 0xFF);
  return s;
}

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}
}  // namespace

V4l2Capture::~V4l2Capture() { stop(); }

void V4l2Capture::start(const CaptureConfig & cfg)
{
  open(cfg);
  startStreaming();
}

void V4l2Capture::open(const CaptureConfig & cfg)
{
  stop();
  device_ = cfg.device;
  requestedBuffers_ = cfg.buffers;
  fd_ = ::open(cfg.device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error("cannot open " + cfg.device + ": " + std::strerror(errno));
  }
  try {
    v4l2_capability cap{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
      throw std::runtime_error(cfg.device + " is not a V4L2 device");
    }
    const uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
    if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) throw std::runtime_error(cfg.device + " does not support video capture");
    if (!(caps & V4L2_CAP_STREAMING)) throw std::runtime_error(cfg.device + " does not support streaming I/O");
    card_ = reinterpret_cast<const char *>(cap.card);

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = cfg.width;
    fmt.fmt.pix.height = cfg.height;
    fmt.fmt.pix.pixelformat = fourccCode(cfg.fourcc);
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
      throw std::runtime_error("VIDIOC_S_FMT failed: " + std::string(std::strerror(errno)));
    }
    if (fmt.fmt.pix.pixelformat != fourccCode(cfg.fourcc)) {
      throw std::runtime_error("device refused pixel format " + cfg.fourcc + ", offered " +
                               fourccString(fmt.fmt.pix.pixelformat));
    }
    if (fmt.fmt.pix.width != cfg.width || fmt.fmt.pix.height != cfg.height) {
      throw std::runtime_error("device refused " + std::to_string(cfg.width) + "x" + std::to_string(cfg.height) +
                               ", offered " + std::to_string(fmt.fmt.pix.width) + "x" +
                               std::to_string(fmt.fmt.pix.height));
    }
    width_ = fmt.fmt.pix.width;
    height_ = fmt.fmt.pix.height;
    stride_ = fmt.fmt.pix.bytesperline ? fmt.fmt.pix.bytesperline : width_ * 2;

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_G_PARM, &parm) == 0 && (parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
      // Express the requested rate as an exact fraction: 59.94 -> 1001/60000.
      uint32_t num = 1, den = static_cast<uint32_t>(std::lround(cfg.fps));
      if (std::fabs(cfg.fps - 59.94) < 0.02) { num = 1001; den = 60000; }
      else if (std::fabs(cfg.fps - 29.97) < 0.02) { num = 1001; den = 30000; }
      parm.parm.capture.timeperframe.numerator = num;
      parm.parm.capture.timeperframe.denominator = den;
      xioctl(fd_, VIDIOC_S_PARM, &parm);  // best effort; the driver reports what it did
      xioctl(fd_, VIDIOC_G_PARM, &parm);
      const auto & tpf = parm.parm.capture.timeperframe;
      fps_ = tpf.numerator ? static_cast<double>(tpf.denominator) / tpf.numerator : cfg.fps;
    } else {
      fps_ = cfg.fps;
    }
  } catch (...) {
    stop();
    throw;
  }
}

void V4l2Capture::startStreaming()
{
  if (fd_ < 0) throw std::runtime_error("video device is not open");
  if (streaming_) return;
  try {
    v4l2_requestbuffers req{};
    req.count = std::max(2u, requestedBuffers_);
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
      throw std::runtime_error("VIDIOC_REQBUFS failed: " + std::string(std::strerror(errno)));
    }
    if (req.count < 2) throw std::runtime_error("insufficient V4L2 buffer memory");
    buffers_.resize(req.count);
    for (uint32_t i = 0; i < req.count; ++i) {
      v4l2_buffer buf{};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;
      if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
        throw std::runtime_error("VIDIOC_QUERYBUF failed: " + std::string(std::strerror(errno)));
      }
      buffers_[i].length = buf.length;
      buffers_[i].start = ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
      if (buffers_[i].start == MAP_FAILED) {
        buffers_[i].start = nullptr;
        throw std::runtime_error("mmap failed: " + std::string(std::strerror(errno)));
      }
    }
    for (uint32_t i = 0; i < req.count; ++i) requeue(i);

    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      throw std::runtime_error("VIDIOC_STREAMON failed: " + std::string(std::strerror(errno)));
    }
    streaming_ = true;
  } catch (...) {
    stop();
    throw;
  }
}

void V4l2Capture::stop()
{
  if (fd_ < 0) return;
  if (streaming_) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
    streaming_ = false;
  }
  for (auto & b : buffers_) {
    if (b.start) ::munmap(b.start, b.length);
  }
  buffers_.clear();
  ::close(fd_);
  fd_ = -1;
}

bool V4l2Capture::dequeue(CapturedFrame * frame, int timeout_ms)
{
  pollfd p{fd_, POLLIN | POLLPRI, 0};
  const int rc = ::poll(&p, 1, timeout_ms);
  if (rc < 0) {
    if (errno == EINTR) return false;
    throw std::runtime_error("poll on video device failed: " + std::string(std::strerror(errno)));
  }
  if (rc == 0) return false;
  if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw std::runtime_error("video device reported an error (unplugged?)");
  }
  v4l2_buffer buf{};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
    if (errno == EAGAIN) return false;
    throw std::runtime_error("VIDIOC_DQBUF failed: " + std::string(std::strerror(errno)));
  }
  frame->index = buf.index;
  frame->data = static_cast<const uint8_t *>(buffers_[buf.index].start);
  frame->bytes = buf.bytesused;
  frame->sequence = buf.sequence;
  frame->timestamp_ns = static_cast<int64_t>(buf.timestamp.tv_sec) * 1000000000LL +
                        static_cast<int64_t>(buf.timestamp.tv_usec) * 1000LL;
  frame->monotonic = (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
  frame->error = (buf.flags & V4L2_BUF_FLAG_ERROR) != 0;
  return true;
}

void V4l2Capture::requeue(uint32_t index)
{
  v4l2_buffer buf{};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = index;
  if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
    throw std::runtime_error("VIDIOC_QBUF failed: " + std::string(std::strerror(errno)));
  }
}

std::string V4l2Capture::usbReset(const std::string & video_path)
{
  // /sys/class/video4linux/videoN/device -> .../usbB-P:C.I ; its parent is the USB device.
  char resolved[PATH_MAX];
  if (!::realpath(video_path.c_str(), resolved)) return "cannot resolve " + video_path;
  const std::string node = std::string(resolved).substr(std::string(resolved).find_last_of('/') + 1);
  std::string sys = "/sys/class/video4linux/" + node + "/device";
  if (!::realpath(sys.c_str(), resolved)) return "no sysfs entry for " + node;
  std::string dev = std::string(resolved) + "/..";
  auto readInt = [&](const std::string & f, int * out) {
    std::ifstream in(dev + "/" + f);
    return static_cast<bool>(in >> *out);
  };
  int bus = 0, num = 0;
  if (!readInt("busnum", &bus) || !readInt("devnum", &num)) return "cannot read USB bus/device number";
  char path[64];
  std::snprintf(path, sizeof(path), "/dev/bus/usb/%03d/%03d", bus, num);
  const int fd = ::open(path, O_WRONLY | O_CLOEXEC);
  if (fd < 0) return std::string(path) + ": " + std::strerror(errno) + " (install the udev rule to allow resets)";
  const int rc = ::ioctl(fd, USBDEVFS_RESET, 0);
  const int err = errno;
  ::close(fd);
  if (rc < 0) return std::string("USBDEVFS_RESET on ") + path + ": " + std::strerror(err);
  return "";
}

DeviceInfo V4l2Capture::query(const std::string & path)
{
  DeviceInfo info;
  info.path = path;
  const int fd = ::open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) return info;
  v4l2_capability cap{};
  if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
    info.card = reinterpret_cast<const char *>(cap.card);
    info.driver = reinterpret_cast<const char *>(cap.driver);
    info.bus_info = reinterpret_cast<const char *>(cap.bus_info);
    const uint32_t caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;
    info.capture = (caps & V4L2_CAP_VIDEO_CAPTURE) && (caps & V4L2_CAP_STREAMING);
    if (info.capture) {
      v4l2_fmtdesc desc{};
      desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      for (desc.index = 0; xioctl(fd, VIDIOC_ENUM_FMT, &desc) == 0; ++desc.index) {
        info.formats.push_back(fourccString(desc.pixelformat));
      }
    }
  }
  ::close(fd);
  return info;
}

std::vector<DeviceInfo> V4l2Capture::listDevices()
{
  std::vector<DeviceInfo> out;
  glob_t g{};
  if (::glob("/dev/video*", 0, nullptr, &g) == 0) {
    for (size_t i = 0; i < g.gl_pathc; ++i) out.push_back(query(g.gl_pathv[i]));
  }
  globfree(&g);
  std::sort(out.begin(), out.end(), [](const DeviceInfo & a, const DeviceInfo & b) {
    // numeric order: video2 before video10
    auto num = [](const std::string & p) {
      size_t k = p.find_last_not_of("0123456789");
      return k == std::string::npos || k + 1 >= p.size() ? 0 : std::stoi(p.substr(k + 1));
    };
    return num(a.path) < num(b.path);
  });
  return out;
}

std::string V4l2Capture::autodetect(const std::vector<std::string> & hints, const std::string & fourcc,
                                    std::string * log)
{
  if (::access("/dev/fcb_video", F_OK) == 0) {
    const auto info = query("/dev/fcb_video");
    if (info.capture) {
      if (log) *log = "using udev symlink /dev/fcb_video (" + info.card + ")";
      return "/dev/fcb_video";
    }
  }
  std::string seen;
  for (const auto & info : listDevices()) {
    seen += info.path + " (" + info.card + (info.capture ? "" : ", no capture") + "); ";
    const std::string card = lower(info.card);
    bool match = hints.empty();
    for (const auto & h : hints) {
      if (!h.empty() && card.find(lower(h)) != std::string::npos) match = true;
    }
    if (!match || !info.capture) continue;
    if (std::find(info.formats.begin(), info.formats.end(), fourcc) == info.formats.end()) continue;
    if (log) *log = "matched \"" + info.card + "\" -> " + info.path;
    return info.path;
  }
  if (log) *log = "no /dev/video* device matched; seen: " + seen;
  return "";
}
}  // namespace fcb_camera
