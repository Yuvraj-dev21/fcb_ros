// VISCA packet encoding/decoding for the Sony FCB-EV9520L block camera.
//
// Byte sequences follow the Sony FCB-EV9520L / EV9500L Technical Manual,
// "Command List". In Sony's notation `8x` is the command header for camera
// address x and `y0` (y = x + 8) is the reply header, so address 1 sends
// 0x81 and hears 0x90 back. Nothing here does I/O; see visca_link.hpp.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace fcb_camera::visca
{
using Packet = std::vector<uint8_t>;

// ---- zoom -------------------------------------------------------------------
constexpr int ZOOM_WIDE_END = 0x0000;
constexpr int ZOOM_OPTICAL_TELE_END = 0x4000;  // 30x
constexpr int ZOOM_DIGITAL_TELE_END = 0x7AC0;  // 30x optical * 12x digital
constexpr double ZOOM_MAX_OPTICAL_RATIO = 30.0;
constexpr double ZOOM_MAX_DIGITAL_RATIO = 360.0;

// Optical magnification <-> raw position, interpolated from the manual's
// "Zoom Ratio and Zoom Position" table. Positions beyond the optical end are
// mapped linearly onto the 1x .. 12x digital range.
int ratioToPosition(double ratio);
double positionToRatio(int position);
int clampZoom(int position, bool allow_digital);

// ---- focus ------------------------------------------------------------------
constexpr int FOCUS_FAR_END = 0x1000;
constexpr int FOCUS_NEAR_END = 0xF000;

// ---- reply classification ---------------------------------------------------
enum class ReplyKind { Ack, Completion, Error, Unknown };
struct Reply
{
  ReplyKind kind{ReplyKind::Unknown};
  Packet payload;  // bytes between reply header and 0xFF terminator
};
Reply classify(const Packet & frame);
std::string errorMessage(const Packet & payload);
std::string hex(const Packet & bytes);
bool parseHex(const std::string & text, Packet * out);

// ---- packet builders -------------------------------------------------------
// All take the VISCA address (1..7) and return the complete frame incl. header and 0xFF.
Packet wrap(uint8_t address, std::initializer_list<uint8_t> body);
Packet wrap(uint8_t address, const Packet & body);

Packet ifClear(uint8_t a);
Packet versionInq(uint8_t a);
Packet power(uint8_t a, bool on);
Packet powerInq(uint8_t a);

Packet zoomStop(uint8_t a);
Packet zoomTele(uint8_t a, int speed);
Packet zoomWide(uint8_t a, int speed);
Packet zoomDirect(uint8_t a, int position);
Packet zoomPosInq(uint8_t a);
Packet digitalZoom(uint8_t a, bool on);
Packet digitalZoomInq(uint8_t a);

Packet focusStop(uint8_t a);
Packet focusFar(uint8_t a, int speed);
Packet focusNear(uint8_t a, int speed);
Packet focusMode(uint8_t a, bool automatic);
Packet focusOnePush(uint8_t a);
Packet focusDirect(uint8_t a, int position);
Packet focusPosInq(uint8_t a);
Packet focusModeInq(uint8_t a);

// ICR: 0x02 on (filter removed, IR), 0x03 off (day), 0x04 on with colour
Packet icr(uint8_t a, uint8_t sub);
Packet icrInq(uint8_t a);
Packet autoIcr(uint8_t a, uint8_t sub);  // 0x02 on, 0x03 off, 0x04 on (colour)
Packet autoIcrInq(uint8_t a);
Packet autoIcrThreshold(uint8_t a, int level);

Packet aeMode(uint8_t a, uint8_t mode);  // 0x00 full auto, 0x03 manual, 0x0A shutter pri, 0x0B iris pri, 0x0D bright
Packet aeModeInq(uint8_t a);
Packet shutterDirect(uint8_t a, int v);
Packet shutterInq(uint8_t a);
Packet irisDirect(uint8_t a, int v);
Packet irisInq(uint8_t a);
Packet gainDirect(uint8_t a, int v);
Packet gainInq(uint8_t a);
Packet gainLimit(uint8_t a, int v);
Packet brightDirect(uint8_t a, int v);
Packet expCompOnOff(uint8_t a, bool on);
Packet expCompDirect(uint8_t a, int v);
Packet backlight(uint8_t a, bool on);
Packet slowShutterAuto(uint8_t a, bool on);

Packet wbMode(uint8_t a, uint8_t mode);
Packet wbModeInq(uint8_t a);
Packet wbOnePushTrigger(uint8_t a);
Packet rGainDirect(uint8_t a, int v);
Packet bGainDirect(uint8_t a, int v);

Packet stabilizer(uint8_t a, bool on);
Packet stabilizerInq(uint8_t a);
Packet defog(uint8_t a, bool on, int level);
Packet noiseReduction(uint8_t a, int level);
Packet apertureDirect(uint8_t a, int v);
Packet pictureFlip(uint8_t a, bool on);
Packet mirror(uint8_t a, bool on);
Packet freeze(uint8_t a, bool on);

Packet memory(uint8_t a, uint8_t action, uint8_t slot);  // 0x00 reset, 0x01 set, 0x02 recall

// ---- reply parsing ---------------------------------------------------------
// Reassemble `count` low nibbles following the 0x50 byte of a completion payload.
int parseNibbles(const Packet & payload, int count);
// Single data byte after 0x50 (mode inquiries), or -1 if the payload is too short.
int parseByte(const Packet & payload);

// ---- name tables -----------------------------------------------------------
std::string aeModeName(int code);
int aeModeCode(const std::string & name);  // -1 if unknown
std::string wbModeName(int code);
int wbModeCode(const std::string & name);
std::string icrModeName(int code);
std::string normalizeIcrMode(const std::string & mode);  // resolves aliases, "" if unknown

}  // namespace fcb_camera::visca
