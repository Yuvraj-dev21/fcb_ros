// Serial transport for VISCA: exclusive, framed, and safe to share between threads.
//
// Every VISCA packet ends in 0xFF. A command is answered with an ACK (y0 4z FF)
// once accepted and a separate completion (y0 5z FF) once the action finished,
// which for a zoom move can be seconds later. `command()` therefore waits only
// for the ACK by default; late completions are skipped by the next exchange.
// Inquiries are answered with a single data-carrying completion (y0 50 .. FF).
#pragma once

#include <chrono>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "fcb_camera/visca.hpp"

namespace fcb_camera
{
class ViscaError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};
class ViscaTimeout : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

class ViscaLink
{
public:
  using Packet = visca::Packet;
  using Duration = std::chrono::milliseconds;

  // Opens the port at `baud`, takes an exclusive flock. Throws std::runtime_error.
  ViscaLink(const std::string & port, int baud, uint8_t address, Duration timeout);
  ~ViscaLink();
  ViscaLink(const ViscaLink &) = delete;
  ViscaLink & operator=(const ViscaLink &) = delete;

  const std::string & port() const { return port_; }
  int baud() const { return baud_; }
  uint8_t address() const { return address_; }

  // Send a command frame; wait for ACK, and for the completion too if requested.
  void command(const Packet & frame, bool wait_completion = false, Duration timeout = Duration(0));
  // Send a command that is answered with a bare completion only (IF_Clear).
  void commandNoAck(const Packet & frame, Duration timeout = Duration(0));
  // Send an inquiry and return the completion payload (0x50 <data...>).
  Packet inquiry(const Packet & frame, Duration timeout = Duration(0));

  // Convenience: true when the port answers a zoom inquiry twice with a well-formed reply.
  static bool probe(const std::string & port, int baud, uint8_t address, Duration timeout);
  // Sweep candidate ports and bauds. Returns {port, baud} or {"", 0}.
  static std::pair<std::string, int> autodetect(
    const std::vector<std::string> & ports, const std::vector<int> & bauds, uint8_t address,
    Duration timeout);
  static std::vector<std::string> defaultPortCandidates();

private:
  Packet readFrame(std::chrono::steady_clock::time_point deadline);
  Packet await(visca::ReplyKind want, Duration timeout, size_t min_payload);
  void write(const Packet & frame);
  void flushInput();

  std::string port_;
  int baud_;
  uint8_t address_;
  Duration timeout_;
  int fd_{-1};
  std::mutex mutex_;
};
}  // namespace fcb_camera
