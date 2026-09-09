#include <gtest/gtest.h>

#include "fcb_camera/visca.hpp"

using namespace fcb_camera::visca;

TEST(Visca, ZoomDirectFrame)
{
  const auto p = zoomDirect(1, 0x2063);
  const Packet expect = {0x81, 0x01, 0x04, 0x47, 0x02, 0x00, 0x06, 0x03, 0xFF};
  EXPECT_EQ(p, expect);
}

TEST(Visca, ZoomRatioRoundTrip)
{
  for (int r = 1; r <= 30; ++r) {
    EXPECT_NEAR(positionToRatio(ratioToPosition(r)), r, 0.01) << "ratio " << r;
  }
  EXPECT_EQ(ratioToPosition(1.0), 0);
  EXPECT_EQ(ratioToPosition(30.0), ZOOM_OPTICAL_TELE_END);
  EXPECT_EQ(ratioToPosition(3.0), 0x2063);
  EXPECT_EQ(ratioToPosition(360.0), ZOOM_DIGITAL_TELE_END);
  EXPECT_NEAR(positionToRatio(ZOOM_DIGITAL_TELE_END), 360.0, 0.01);
  EXPECT_EQ(clampZoom(0x5000, false), ZOOM_OPTICAL_TELE_END);
  EXPECT_EQ(clampZoom(0x5000, true), 0x5000);
}

TEST(Visca, ClassifyReplies)
{
  EXPECT_EQ(classify({0x90, 0x41, 0xFF}).kind, ReplyKind::Ack);
  EXPECT_EQ(classify({0x90, 0x51, 0xFF}).kind, ReplyKind::Completion);
  const auto inq = classify({0x90, 0x50, 0x02, 0x00, 0x06, 0x03, 0xFF});
  EXPECT_EQ(inq.kind, ReplyKind::Completion);
  EXPECT_EQ(parseNibbles(inq.payload, 4), 0x2063);
  const auto err = classify({0x90, 0x60, 0x41, 0xFF});
  EXPECT_EQ(err.kind, ReplyKind::Error);
  EXPECT_EQ(errorMessage(err.payload), "command not executable");
  EXPECT_EQ(classify({0x90}).kind, ReplyKind::Unknown);
}

TEST(Visca, HexParsing)
{
  Packet out;
  EXPECT_TRUE(parseHex("01 04 07 00", &out));
  EXPECT_EQ(out, (Packet{0x01, 0x04, 0x07, 0x00}));
  EXPECT_TRUE(parseHex("01040700", &out));
  EXPECT_EQ(out.size(), 4u);
  EXPECT_TRUE(parseHex("0x01,0x04", &out));
  EXPECT_EQ(out, (Packet{0x01, 0x04}));
  EXPECT_FALSE(parseHex("0104 7", &out));
  EXPECT_FALSE(parseHex("zz", &out));
  EXPECT_EQ(hex({0xDE, 0xAD}), "de ad");
}

TEST(Visca, NameTables)
{
  EXPECT_EQ(normalizeIcrMode("IR"), "night");
  EXPECT_EQ(normalizeIcrMode("rgb"), "day");
  EXPECT_EQ(normalizeIcrMode("bogus"), "");
  EXPECT_EQ(aeModeCode("shutter_priority"), 0x0A);
  EXPECT_EQ(aeModeName(0x0D), "bright");
  EXPECT_EQ(wbModeCode("atw"), 4);
  EXPECT_EQ(wbModeName(9), "sodium_outdoor_auto");
  EXPECT_EQ(icrModeName(0x04), "night_color");
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
