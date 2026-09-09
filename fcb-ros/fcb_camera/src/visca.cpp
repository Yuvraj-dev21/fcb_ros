#include "fcb_camera/visca.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <sstream>

namespace fcb_camera::visca
{
namespace
{
// (magnification, position) from the manual's reference table.
const std::vector<std::pair<double, int>> kRatioTable = {
  {1, 0x0000},  {2, 0x16A1},  {3, 0x2063},  {4, 0x2628},  {5, 0x2A1D},  {6, 0x2D13},
  {7, 0x2F6D},  {8, 0x3161},  {9, 0x330D},  {10, 0x3486}, {11, 0x35D7}, {12, 0x3709},
  {13, 0x3820}, {14, 0x3920}, {15, 0x3A0A}, {16, 0x3ADD}, {17, 0x3B9C}, {18, 0x3C46},
  {19, 0x3CDC}, {20, 0x3D60}, {21, 0x3DD4}, {22, 0x3E39}, {23, 0x3E90}, {24, 0x3EDC},
  {25, 0x3F1E}, {26, 0x3F57}, {27, 0x3F8A}, {28, 0x3FB6}, {29, 0x3FDC}, {30, 0x4000},
};

std::vector<uint8_t> nibbles(int value, int count)
{
  std::vector<uint8_t> out;
  for (int i = 0; i < count; ++i) {
    out.push_back(static_cast<uint8_t>((value >> (4 * (count - 1 - i))) & 0x0F));
  }
  return out;
}

int clampi(int v, int lo, int hi) { return std::max(lo, std::min(hi, v)); }

Packet fourNibble(uint8_t a, uint8_t cmd, int value)
{
  Packet body = {0x01, 0x04, cmd};
  const auto n = nibbles(clampi(value, 0, 0xFFFF), 4);
  body.insert(body.end(), n.begin(), n.end());
  return wrap(a, body);
}

Packet twoNibblePadded(uint8_t a, uint8_t cmd, int value)
{
  Packet body = {0x01, 0x04, cmd, 0x00, 0x00};
  const auto n = nibbles(clampi(value, 0, 0xFF), 2);
  body.insert(body.end(), n.begin(), n.end());
  return wrap(a, body);
}

Packet onOff(uint8_t a, uint8_t cmd, bool on)
{
  return wrap(a, {0x01, 0x04, cmd, static_cast<uint8_t>(on ? 0x02 : 0x03)});
}

Packet inq(uint8_t a, uint8_t cmd) { return wrap(a, {0x09, 0x04, cmd}); }

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}
}  // namespace

// ---- zoom mapping ----------------------------------------------------------

int ratioToPosition(double ratio)
{
  if (std::isnan(ratio)) return 0;
  ratio = std::max(1.0, std::min(ZOOM_MAX_DIGITAL_RATIO, ratio));
  if (ratio > ZOOM_MAX_OPTICAL_RATIO) {
    const double digital = ratio / ZOOM_MAX_OPTICAL_RATIO;  // 1 .. 12
    const double frac = (digital - 1.0) / 11.0;
    return static_cast<int>(std::lround(
      ZOOM_OPTICAL_TELE_END + frac * (ZOOM_DIGITAL_TELE_END - ZOOM_OPTICAL_TELE_END)));
  }
  for (size_t i = 1; i < kRatioTable.size(); ++i) {
    const auto & [hi_r, hi_p] = kRatioTable[i];
    if (ratio <= hi_r) {
      const auto & [lo_r, lo_p] = kRatioTable[i - 1];
      const double frac = (ratio - lo_r) / (hi_r - lo_r);
      return static_cast<int>(std::lround(lo_p + frac * (hi_p - lo_p)));
    }
  }
  return ZOOM_OPTICAL_TELE_END;
}

double positionToRatio(int position)
{
  position = clampi(position, ZOOM_WIDE_END, ZOOM_DIGITAL_TELE_END);
  if (position > ZOOM_OPTICAL_TELE_END) {
    const double frac = static_cast<double>(position - ZOOM_OPTICAL_TELE_END) /
                        (ZOOM_DIGITAL_TELE_END - ZOOM_OPTICAL_TELE_END);
    return ZOOM_MAX_OPTICAL_RATIO * (1.0 + 11.0 * frac);
  }
  for (size_t i = 1; i < kRatioTable.size(); ++i) {
    const auto & [hi_r, hi_p] = kRatioTable[i];
    if (position <= hi_p) {
      const auto & [lo_r, lo_p] = kRatioTable[i - 1];
      const double frac = static_cast<double>(position - lo_p) / (hi_p - lo_p);
      return lo_r + frac * (hi_r - lo_r);
    }
  }
  return ZOOM_MAX_OPTICAL_RATIO;
}

int clampZoom(int position, bool allow_digital)
{
  return clampi(position, ZOOM_WIDE_END, allow_digital ? ZOOM_DIGITAL_TELE_END : ZOOM_OPTICAL_TELE_END);
}

// ---- framing ---------------------------------------------------------------

Packet wrap(uint8_t address, std::initializer_list<uint8_t> body) { return wrap(address, Packet(body)); }

Packet wrap(uint8_t address, const Packet & body)
{
  Packet p;
  p.reserve(body.size() + 2);
  p.push_back(static_cast<uint8_t>(0x80 | (address & 0x07)));
  p.insert(p.end(), body.begin(), body.end());
  p.push_back(0xFF);
  return p;
}

Reply classify(const Packet & frame)
{
  Reply r;
  if (frame.size() < 3 || frame.back() != 0xFF) return r;
  r.payload.assign(frame.begin() + 1, frame.end() - 1);
  switch (r.payload[0] & 0xF0) {
    case 0x40: r.kind = ReplyKind::Ack; break;
    case 0x50: r.kind = ReplyKind::Completion; break;
    case 0x60: r.kind = ReplyKind::Error; break;
    default: r.kind = ReplyKind::Unknown; break;
  }
  return r;
}

std::string errorMessage(const Packet & payload)
{
  static const std::map<int, std::string> codes = {
    {0x01, "message length error"}, {0x02, "syntax error"},
    {0x03, "command buffer full"},   {0x04, "command cancelled"},
    {0x05, "no socket"},             {0x41, "command not executable"},
  };
  if (payload.size() < 2) return "malformed error reply";
  auto it = codes.find(payload[1]);
  if (it != codes.end()) return it->second;
  std::ostringstream ss;
  ss << "unknown VISCA error 0x" << std::hex << static_cast<int>(payload[1]);
  return ss.str();
}

std::string hex(const Packet & bytes)
{
  static const char * digits = "0123456789abcdef";
  std::string s;
  for (size_t i = 0; i < bytes.size(); ++i) {
    if (i) s += ' ';
    s += digits[bytes[i] >> 4];
    s += digits[bytes[i] & 0x0F];
  }
  return s;
}

bool parseHex(const std::string & text, Packet * out)
{
  out->clear();
  int pending = -1;
  for (char c : text) {
    if (std::isspace(static_cast<unsigned char>(c)) || c == ',' || c == ':') {
      if (pending >= 0) return false;  // lone nibble
      continue;
    }
    int v;
    if (c >= '0' && c <= '9') v = c - '0';
    else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
    else if (c == 'x' || c == 'X') { pending = -1; continue; }  // allow 0x prefixes
    else return false;
    if (pending < 0) pending = v;
    else { out->push_back(static_cast<uint8_t>((pending << 4) | v)); pending = -1; }
  }
  return pending < 0;
}

// ---- builders --------------------------------------------------------------

Packet ifClear(uint8_t a) { return wrap(a, {0x01, 0x00, 0x01}); }
Packet versionInq(uint8_t a) { return wrap(a, {0x09, 0x00, 0x02}); }
Packet power(uint8_t a, bool on) { return onOff(a, 0x00, on); }
Packet powerInq(uint8_t a) { return inq(a, 0x00); }

Packet zoomStop(uint8_t a) { return wrap(a, {0x01, 0x04, 0x07, 0x00}); }
Packet zoomTele(uint8_t a, int s) { return wrap(a, {0x01, 0x04, 0x07, static_cast<uint8_t>(0x20 | (clampi(s, 0, 7)))}); }
Packet zoomWide(uint8_t a, int s) { return wrap(a, {0x01, 0x04, 0x07, static_cast<uint8_t>(0x30 | (clampi(s, 0, 7)))}); }
Packet zoomDirect(uint8_t a, int p) { return fourNibble(a, 0x47, clampi(p, 0, ZOOM_DIGITAL_TELE_END)); }
Packet zoomPosInq(uint8_t a) { return inq(a, 0x47); }
Packet digitalZoom(uint8_t a, bool on) { return onOff(a, 0x06, on); }
Packet digitalZoomInq(uint8_t a) { return inq(a, 0x06); }

Packet focusStop(uint8_t a) { return wrap(a, {0x01, 0x04, 0x08, 0x00}); }
Packet focusFar(uint8_t a, int s) { return wrap(a, {0x01, 0x04, 0x08, static_cast<uint8_t>(0x20 | clampi(s, 0, 7))}); }
Packet focusNear(uint8_t a, int s) { return wrap(a, {0x01, 0x04, 0x08, static_cast<uint8_t>(0x30 | clampi(s, 0, 7))}); }
Packet focusMode(uint8_t a, bool automatic) { return onOff(a, 0x38, automatic); }
Packet focusOnePush(uint8_t a) { return wrap(a, {0x01, 0x04, 0x18, 0x01}); }
Packet focusDirect(uint8_t a, int p) { return fourNibble(a, 0x48, p); }
Packet focusPosInq(uint8_t a) { return inq(a, 0x48); }
Packet focusModeInq(uint8_t a) { return inq(a, 0x38); }

Packet icr(uint8_t a, uint8_t sub) { return wrap(a, {0x01, 0x04, 0x01, sub}); }
Packet icrInq(uint8_t a) { return inq(a, 0x01); }
Packet autoIcr(uint8_t a, uint8_t sub) { return wrap(a, {0x01, 0x04, 0x51, sub}); }
Packet autoIcrInq(uint8_t a) { return inq(a, 0x51); }
Packet autoIcrThreshold(uint8_t a, int level) { return twoNibblePadded(a, 0x21, level); }

Packet aeMode(uint8_t a, uint8_t mode) { return wrap(a, {0x01, 0x04, 0x39, mode}); }
Packet aeModeInq(uint8_t a) { return inq(a, 0x39); }
Packet shutterDirect(uint8_t a, int v) { return twoNibblePadded(a, 0x4A, v); }
Packet shutterInq(uint8_t a) { return inq(a, 0x4A); }
Packet irisDirect(uint8_t a, int v) { return twoNibblePadded(a, 0x4B, v); }
Packet irisInq(uint8_t a) { return inq(a, 0x4B); }
Packet gainDirect(uint8_t a, int v) { return twoNibblePadded(a, 0x4C, v); }
Packet gainInq(uint8_t a) { return inq(a, 0x4C); }
Packet gainLimit(uint8_t a, int v) { return wrap(a, {0x01, 0x04, 0x2C, static_cast<uint8_t>(clampi(v, 0, 15))}); }
Packet brightDirect(uint8_t a, int v) { return twoNibblePadded(a, 0x4D, v); }
Packet expCompOnOff(uint8_t a, bool on) { return onOff(a, 0x3E, on); }
Packet expCompDirect(uint8_t a, int v) { return twoNibblePadded(a, 0x4E, v); }
Packet backlight(uint8_t a, bool on) { return onOff(a, 0x33, on); }
Packet slowShutterAuto(uint8_t a, bool on) { return onOff(a, 0x5A, on); }

Packet wbMode(uint8_t a, uint8_t mode) { return wrap(a, {0x01, 0x04, 0x35, mode}); }
Packet wbModeInq(uint8_t a) { return inq(a, 0x35); }
Packet wbOnePushTrigger(uint8_t a) { return wrap(a, {0x01, 0x04, 0x10, 0x05}); }
Packet rGainDirect(uint8_t a, int v) { return twoNibblePadded(a, 0x43, v); }
Packet bGainDirect(uint8_t a, int v) { return twoNibblePadded(a, 0x44, v); }

Packet stabilizer(uint8_t a, bool on) { return onOff(a, 0x34, on); }
Packet stabilizerInq(uint8_t a) { return inq(a, 0x34); }
Packet defog(uint8_t a, bool on, int level)
{
  if (on) return wrap(a, {0x01, 0x04, 0x37, 0x02, static_cast<uint8_t>(clampi(level, 1, 3))});
  return wrap(a, {0x01, 0x04, 0x37, 0x03, 0x00});
}
Packet noiseReduction(uint8_t a, int level) { return wrap(a, {0x01, 0x04, 0x53, static_cast<uint8_t>(clampi(level, 0, 5))}); }
Packet apertureDirect(uint8_t a, int v) { return twoNibblePadded(a, 0x42, clampi(v, 0, 15)); }
Packet pictureFlip(uint8_t a, bool on) { return onOff(a, 0x66, on); }
Packet mirror(uint8_t a, bool on) { return onOff(a, 0x61, on); }
Packet freeze(uint8_t a, bool on) { return onOff(a, 0x62, on); }

Packet memory(uint8_t a, uint8_t action, uint8_t slot)
{
  return wrap(a, {0x01, 0x04, 0x3F, action, static_cast<uint8_t>(slot & 0x0F)});
}

// ---- parsing ---------------------------------------------------------------

int parseNibbles(const Packet & payload, int count)
{
  if (static_cast<int>(payload.size()) < count + 1) return -1;
  int value = 0;
  for (int i = 1; i <= count; ++i) value = (value << 4) | (payload[i] & 0x0F);
  return value;
}

int parseByte(const Packet & payload) { return payload.size() >= 2 ? payload[1] : -1; }

// ---- name tables -----------------------------------------------------------

std::string aeModeName(int code)
{
  switch (code) {
    case 0x00: return "full_auto";
    case 0x03: return "manual";
    case 0x0A: return "shutter_priority";
    case 0x0B: return "iris_priority";
    case 0x0D: return "bright";
    default: return "unknown";
  }
}

int aeModeCode(const std::string & name)
{
  static const std::map<std::string, int> m = {
    {"full_auto", 0x00}, {"auto", 0x00},          {"manual", 0x03},
    {"shutter_priority", 0x0A}, {"shutter", 0x0A}, {"iris_priority", 0x0B},
    {"iris", 0x0B},      {"bright", 0x0D},
  };
  auto it = m.find(lower(name));
  return it == m.end() ? -1 : it->second;
}

std::string wbModeName(int code)
{
  static const char * names[] = {"auto", "indoor", "outdoor", "one_push", "atw", "manual",
                                 "outdoor_auto", "sodium_auto", "sodium", "sodium_outdoor_auto"};
  if (code >= 0 && code < 10) return names[code];
  return "unknown";
}

int wbModeCode(const std::string & name)
{
  const std::string n = lower(name);
  for (int i = 0; i < 10; ++i) {
    if (wbModeName(i) == n) return i;
  }
  return -1;
}

std::string icrModeName(int code)
{
  switch (code) {
    case 0x02: return "night";
    case 0x03: return "day";
    case 0x04: return "night_color";
    default: return "unknown";
  }
}

std::string normalizeIcrMode(const std::string & mode)
{
  static const std::map<std::string, std::string> aliases = {
    {"day", "day"},           {"rgb", "day"},       {"color", "day"},
    {"colour", "day"},        {"night", "night"},   {"ir", "night"},
    {"bw", "night"},          {"mono", "night"},    {"night_color", "night_color"},
    {"ir_color", "night_color"}, {"auto", "auto"},  {"auto_color", "auto_color"},
    {"manual", "manual"},
  };
  auto it = aliases.find(lower(mode));
  return it == aliases.end() ? "" : it->second;
}

}  // namespace fcb_camera::visca
