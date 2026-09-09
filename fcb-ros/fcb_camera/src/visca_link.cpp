#include "fcb_camera/visca_link.hpp"

#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <thread>

namespace fcb_camera
{
namespace
{
speed_t baudConstant(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: throw std::runtime_error("unsupported VISCA baud rate " + std::to_string(baud));
  }
}

std::vector<std::string> globPaths(const char * pattern)
{
  std::vector<std::string> out;
  glob_t g{};
  if (::glob(pattern, 0, nullptr, &g) == 0) {
    for (size_t i = 0; i < g.gl_pathc; ++i) out.emplace_back(g.gl_pathv[i]);
  }
  globfree(&g);
  return out;
}
}  // namespace

ViscaLink::ViscaLink(const std::string & port, int baud, uint8_t address, Duration timeout)
: port_(port), baud_(baud), address_(address), timeout_(timeout)
{
  const speed_t speed = baudConstant(baud);
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error("cannot open " + port + ": " + std::strerror(errno));
  }
  if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error(port + " is locked by another process; refusing to share the VISCA bus");
  }
  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcgetattr " + port + ": " + err);
  }
  cfmakeraw(&tio);
  tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
  tio.c_cflag |= CS8 | CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    const std::string err = std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcsetattr " + port + ": " + err);
  }
  ::tcflush(fd_, TCIOFLUSH);
}

ViscaLink::~ViscaLink()
{
  if (fd_ >= 0) {
    ::flock(fd_, LOCK_UN);
    ::close(fd_);
  }
}

void ViscaLink::flushInput() { ::tcflush(fd_, TCIFLUSH); }

void ViscaLink::write(const Packet & frame)
{
  size_t off = 0;
  while (off < frame.size()) {
    const ssize_t n = ::write(fd_, frame.data() + off, frame.size() - off);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        pollfd p{fd_, POLLOUT, 0};
        ::poll(&p, 1, 100);
        continue;
      }
      throw std::runtime_error(std::string("VISCA write failed: ") + std::strerror(errno));
    }
    off += static_cast<size_t>(n);
  }
  ::tcdrain(fd_);
}

ViscaLink::Packet ViscaLink::readFrame(std::chrono::steady_clock::time_point deadline)
{
  Packet frame;
  for (;;) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      throw ViscaTimeout("no complete VISCA frame within timeout (partial: " + visca::hex(frame) + ")");
    }
    const int ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
    pollfd p{fd_, POLLIN, 0};
    const int rc = ::poll(&p, 1, std::max(1, ms));
    if (rc < 0) {
      if (errno == EINTR) continue;
      throw std::runtime_error(std::string("VISCA poll failed: ") + std::strerror(errno));
    }
    if (rc == 0) continue;
    uint8_t byte;
    const ssize_t n = ::read(fd_, &byte, 1);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      throw std::runtime_error(std::string("VISCA read failed: ") + std::strerror(errno));
    }
    if (n == 0) continue;
    frame.push_back(byte);
    if (byte == 0xFF) return frame;
    if (frame.size() > 64) frame.clear();  // garbage without terminator; resync
  }
}

ViscaLink::Packet ViscaLink::await(visca::ReplyKind want, Duration timeout, size_t min_payload)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  for (;;) {
    const Packet frame = readFrame(deadline);
    const visca::Reply reply = visca::classify(frame);
    if (reply.kind == visca::ReplyKind::Error) {
      throw ViscaError(visca::errorMessage(reply.payload));
    }
    if (reply.kind == want && reply.payload.size() >= min_payload) {
      return reply.payload;
    }
    // An ACK we are not waiting on, or a late completion of an earlier
    // movement: drop it and keep reading.
  }
}

void ViscaLink::command(const Packet & frame, bool wait_completion, Duration timeout)
{
  if (timeout.count() <= 0) timeout = timeout_;
  std::lock_guard<std::mutex> lock(mutex_);
  write(frame);
  await(visca::ReplyKind::Ack, timeout_, 1);
  if (wait_completion) await(visca::ReplyKind::Completion, timeout, 1);
}

void ViscaLink::commandNoAck(const Packet & frame, Duration timeout)
{
  if (timeout.count() <= 0) timeout = timeout_;
  std::lock_guard<std::mutex> lock(mutex_);
  write(frame);
  await(visca::ReplyKind::Completion, timeout, 1);
}

ViscaLink::Packet ViscaLink::inquiry(const Packet & frame, Duration timeout)
{
  if (timeout.count() <= 0) timeout = timeout_;
  std::lock_guard<std::mutex> lock(mutex_);
  write(frame);
  // A bare command completion (y0 5z) has a one-byte payload; an inquiry reply
  // carries data, so insist on at least two bytes to skip stale completions.
  return await(visca::ReplyKind::Completion, timeout, 2);
}

bool ViscaLink::probe(const std::string & port, int baud, uint8_t address, Duration timeout)
{
  try {
    ViscaLink link(port, baud, address, timeout);
    for (int i = 0; i < 2; ++i) {
      const Packet payload = link.inquiry(visca::zoomPosInq(address));
      // A zoom reply is exactly 50 0p 0q 0r 0s: every data byte <= 0x0F. This
      // shape check is what MAVLink noise on a neighbouring port fails.
      if (payload.size() != 5 || payload[0] != 0x50) return false;
      for (size_t k = 1; k < 5; ++k) {
        if (payload[k] > 0x0F) return false;
      }
      const int pos = visca::parseNibbles(payload, 4);
      if (pos < 0 || pos > visca::ZOOM_DIGITAL_TELE_END) return false;
    }
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

std::pair<std::string, int> ViscaLink::autodetect(
  const std::vector<std::string> & ports, const std::vector<int> & bauds, uint8_t address,
  Duration timeout)
{
  for (const auto & port : ports) {
    for (int baud : bauds) {
      if (probe(port, baud, address, timeout)) return {port, baud};
    }
  }
  return {"", 0};
}

std::vector<std::string> ViscaLink::defaultPortCandidates()
{
  std::vector<std::string> out;
  if (::access("/dev/fcb_visca", F_OK) == 0) out.push_back("/dev/fcb_visca");
  for (const char * pattern : {"/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyTHS*"}) {
    for (auto & p : globPaths(pattern)) out.push_back(p);
  }
  return out;
}
}  // namespace fcb_camera
