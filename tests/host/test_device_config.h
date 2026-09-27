// Settings the page can change, and the commands built from them.
#pragma once
#include <cmath>
#include <cstring>
using devset::DriverConfig;
using devset::LoopConfig;

TEST(DC_LoopConfig_DefaultsAreValid) {
  CHECK(devset::validateLoop(LoopConfig()) == nullptr);
  CHECK(devset::validateDriver(DriverConfig()) == nullptr);
}

TEST(DC_LoopConfig_RejectsEachFieldOutOfRange) {
  LoopConfig c;
  c.factorX = 0;             CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.factorY = 100001;        CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.toleranceArcmin = 0.09;  CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.toleranceArcmin = 10.01; CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.userCapArcmin = 0.99;    CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.userCapArcmin = 60.01;   CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.settleMs = 30001;        CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.feed = 49;               CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.feed = 3001;             CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.factorX = std::nan("");  CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.calThresholdArcmin = 0.99;  CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.calThresholdArcmin = 20.01; CHECK(devset::validateLoop(c) != nullptr); c = LoopConfig();
  c.factorX = 60; c.factorY = 180; c.toleranceArcmin = 0.1; c.userCapArcmin = 60; c.settleMs = 30000; c.feed = 3000;
  CHECK(devset::validateLoop(c) == nullptr);
}

TEST(DC_ValidateHost_EmptyOrDottedIpv4) {
  for (const char *ok : {"", "192.168.1.53", "10.0.0.1", "255.255.255.255"}) CHECK(devset::validateHost(ok) == nullptr);
  for (const char *bad : {"192.168.1", "192.168.1.256", "1.2.3.4.5", "a.b.c.d", "192.168.1.", "1..2.3", "1234.1.1.1",
                          " 1.2.3.4", "asiair.local"}) {
    CHECK(devset::validateHost(bad) != nullptr);
  }
  CHECK(devset::validateHost(nullptr) != nullptr);
}

TEST(DC_DriverConfig_RejectsOutOfRange) {
  DriverConfig d;
  d.runMa = 99;      CHECK(devset::validateDriver(d) != nullptr); d = DriverConfig();
  d.runMa = 2001;    CHECK(devset::validateDriver(d) != nullptr); d = DriverConfig();
  d.holdPct = 101;   CHECK(devset::validateDriver(d) != nullptr); d = DriverConfig();
  d.microsteps = 12; CHECK(devset::validateDriver(d) != nullptr); d = DriverConfig();
  d.microsteps = 256; d.holdPct = 0; d.runMa = 2000;
  CHECK(devset::validateDriver(d) == nullptr);
}

TEST(DC_DriverCommands_MicrostepsFirst_TypeFirstGrammar) {
  char out[3][12];
  DriverConfig d;
  d.runMa = 700; d.holdPct = 40; d.microsteps = 4;
  CHECK(devset::driverCommands('Y', d, out));
  CHECK_EQ_STR(out[0], "SY4");
  CHECK_EQ_STR(out[1], "CY700");
  CHECK_EQ_STR(out[2], "HY40");
  CHECK(!devset::driverCommands('Z', d, out));
  d.runMa = 5000;
  CHECK(!devset::driverCommands('X', d, out));
}

TEST(DC_MoveCommand_ConvertsArcminWithTheFactor) {
  char buf[48];
  CHECK(devset::moveCommand('X', 1.5, 60, 1000, buf, sizeof buf));
  CHECK_EQ_STR(buf, "$J=G91G21X90F1000");
  CHECK(devset::moveCommand('Y', -5, 60.4, 800, buf, sizeof buf));
  CHECK_EQ_STR(buf, "$J=G91G21Y-302F800");
}

TEST(DC_MoveCommand_RejectsZeroStepsAndBadInput) {
  char buf[48];
  CHECK(!devset::moveCommand('X', 0.004, 60, 1000, buf, sizeof buf));  // rounds to 0 steps
  CHECK(!devset::moveCommand('X', 1, 0, 1000, buf, sizeof buf));
  CHECK(!devset::moveCommand('X', std::nan(""), 60, 1000, buf, sizeof buf));
  CHECK(!devset::moveCommand('X', 1e9, 60, 1000, buf, sizeof buf));
}
