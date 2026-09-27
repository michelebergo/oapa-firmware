// Web-side rules: what a phone may inject and when N.I.N.A. owns the motors.
#pragma once
#include <cstring>

TEST(Whitelist_AcceptsJogsBareMovesAndStop) {
  for (const char *c : {"$J=G91G21X800F1000", "$J=G53Y42F800", "X800", "Y-200", "!"}) {
    CHECK(web::isAllowedCommand(c));
  }
}

TEST(Whitelist_RefusesEverythingThatWritesSerialOrConfiguresDrivers) {
  for (const char *c : {"?", "$H", "CX600", "HX50", "SX16", "XC600", "", "Z1", "!!", "$J=G91\nX1"}) {
    CHECK(!web::isAllowedCommand(c));
  }
  CHECK(!web::isAllowedCommand(nullptr));
  std::string tooLong = "$J=G91G21X" + std::string(60, '1');
  CHECK(!web::isAllowedCommand(tooLong.c_str()));
}

TEST(BuildJog_FormatsTheSameGrammarAsThePlugin) {
  char buf[web::kCommandLen];
  CHECK(web::buildJogCommand('X', 800, 1000, buf, sizeof buf));
  CHECK_EQ_STR(buf, "$J=G91G21X800F1000");
  CHECK(web::buildJogCommand('Y', -25, 50, buf, sizeof buf));
  CHECK_EQ_STR(buf, "$J=G91G21Y-25F50");
}

TEST(BuildJog_RejectsOutOfRangeInput) {
  char buf[web::kCommandLen];
  CHECK(!web::buildJogCommand('Z', 800, 1000, buf, sizeof buf));
  CHECK(!web::buildJogCommand('X', 0, 1000, buf, sizeof buf));
  CHECK(!web::buildJogCommand('X', 200001, 1000, buf, sizeof buf));
  CHECK(!web::buildJogCommand('X', -200001, 1000, buf, sizeof buf));
  CHECK(!web::buildJogCommand('X', 800, 49, buf, sizeof buf));
  CHECK(!web::buildJogCommand('X', 800, 3001, buf, sizeof buf));
  char tiny[8];
  CHECK(!web::buildJogCommand('X', 800, 1000, tiny, sizeof tiny));
}

TEST(NinaActive_TenSecondWindow_SurvivesMillisWrap) {
  CHECK(!web::ninaActive(false, 5000, 0));
  CHECK(web::ninaActive(true, 1000, 1000));
  CHECK(web::ninaActive(true, 10999, 1000));
  CHECK(!web::ninaActive(true, 11000, 1000));
  CHECK(web::ninaActive(true, 0x00000100u, 0xFFFFF000u));  // 4352 ms across wrap
}

TEST(RunWebCommand_RefusesStatusProbe_WithoutWritingSerial) {
  resetFirmwareState();
  CHECK(!runWebCommand("?"));
  CHECK_EQ_STR(Serial.out, "");
}

TEST(RunWebCommand_RefusesHoming) {
  resetFirmwareState();
  xAxis.stepper.setCurrentPosition(123);
  CHECK(!runWebCommand("$H"));
  CHECK(xAxis.stepper.currentPosition() == 123);
  CHECK_EQ_STR(Serial.out, "");
}

TEST(RunWebCommand_BuiltJog_MovesLikeTheSerialPath_AndStaysSilent) {
  resetFirmwareState();
  char buf[web::kCommandLen];
  web::buildJogCommand('Y', -300, 700, buf, sizeof buf);
  CHECK(runWebCommand(buf));
  CHECK(yAxis.stepper.targetPosition() == -300);
  CHECK_NEAR(yAxis.stepper.maxSpeed(), 700, 0.001);
  CHECK(runWebCommand("!"));
  CHECK(!yAxis.stepper.isRunning());
  CHECK_EQ_STR(Serial.out, "");
}

TEST(WB_DriverCommand_OnlyTypeFirstGrammar) {
  for (const char *c : {"CX600", "HY40", "SX16", "SY256"}) CHECK(web::isDriverCommand(c));
  for (const char *c : {"XC600", "CZ600", "CX", "cx600", "CX6000000", "?", "$J=G91G21X10F800", "CX60a"}) {
    CHECK(!web::isDriverCommand(c));
  }
  CHECK(!web::isDriverCommand(nullptr));
}

TEST(WB_RunWebDriverCommand_AppliesAndStaysSilent) {
  resetFirmwareState();
  CHECK(runWebDriverCommand("CX900"));
  CHECK(xAxis.runCurrent_mA == 900 && tmcX.lastRunMa == 900);
  CHECK(runWebDriverCommand("SY4"));
  CHECK(yAxis.microsteps == 4);
  CHECK(!runWebDriverCommand("$J=G91G21X10F800"));
  CHECK(xAxis.stepper.targetPosition() == 0);
  CHECK_EQ_STR(Serial.out, "");
}
