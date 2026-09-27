// Port of nina.plugin.polaralignment OapaFirmwareWireContractTest, run against
// the verbatim 1.2.2 dispatcher instead of a C# re-implementation.
#pragma once

static String run(const char *command) { return dispatchCommand(String(command)); }

TEST(StatusProbe_WritesExactlyFrameThenOk) {
  resetFirmwareState();
  CHECK_EQ_STR(run("?").c_str(), "");
  CHECK_EQ_STR(Serial.out, "<Idle|MPos:0.00,0.00,0.00|V:1.3.0|>\r\nok\r\n");
}

TEST(StatusProbe_ReportsRunWhileMoving) {
  resetFirmwareState();
  run("X800");
  Serial.out.clear();
  run("?");
  CHECK(Serial.out.rfind("<Run|MPos:", 0) == 0);
}

TEST(RelativeJog_AsBuiltByMoveRelative_MovesRoundedAndUsesFeed) {
  resetFirmwareState();
  CHECK_EQ_STR(run("$J=G91G21X-1234.567F800").c_str(), "ok");
  CHECK(xAxis.stepper.targetPosition() == -1235);
  CHECK_NEAR(xAxis.stepper.maxSpeed(), 800, 0.001);
}

TEST(AbsoluteJog_AsBuiltByMoveAbsolute_MovesRoundedAndUsesFeed) {
  resetFirmwareState();
  CHECK_EQ_STR(run("$J=G53Y42.5F800").c_str(), "ok");
  CHECK(yAxis.stepper.targetPosition() == 43);
  CHECK_NEAR(yAxis.stepper.maxSpeed(), 800, 0.001);
}

TEST(DirectMove_X800_And_YMinus200_AreRelativeAtDefaultSpeed) {
  resetFirmwareState();
  run("$J=G91G21X1F800");  // leaves F800 behind; bare moves must reset it
  xAxis.stepper.setCurrentPosition(0);
  CHECK_EQ_STR(run("X800").c_str(), "ok");
  CHECK(xAxis.stepper.targetPosition() == 800);
  CHECK_NEAR(xAxis.stepper.maxSpeed(), DEFAULT_MAX_SPEED, 0.001);
  CHECK_EQ_STR(run("Y-200").c_str(), "ok");
  CHECK(yAxis.stepper.targetPosition() == -200);
}

TEST(RunCurrent_CX1200_CY1200_ReachDrivers) {
  resetFirmwareState();
  CHECK_EQ_STR(run("CX1200").c_str(), "ok");
  CHECK(xAxis.runCurrent_mA == 1200 && tmcX.lastRunMa == 1200);
  CHECK_EQ_STR(run("CY1200").c_str(), "ok");
  CHECK(yAxis.runCurrent_mA == 1200 && tmcY.lastRunMa == 1200);
}

TEST(HoldPercent_HX40_HY40_ReachDrivers) {
  resetFirmwareState();
  run("HX40");
  run("HY40");
  CHECK_NEAR(xAxis.holdMultiplier, 0.40, 1e-6);
  CHECK_NEAR(yAxis.holdMultiplier, 0.40, 1e-6);
  CHECK_NEAR(tmcY.lastHold, 0.40, 1e-6);
}

TEST(Microsteps_SX4_SY4_ReachDrivers) {
  resetFirmwareState();
  run("SX4");
  run("SY4");
  CHECK(xAxis.microsteps == 4 && tmcX.lastMicrosteps == 4);
  CHECK(yAxis.microsteps == 4 && tmcY.lastMicrosteps == 4);
}

TEST(StartupBatch_AsPushedOnConnect_IsFullyApplied) {
  resetFirmwareState();
  // OapaDriverCommands.StartupBatch(600, 50, 700, 40, 16, 4)
  for (const char *c : {"SX16", "SY4", "CX600", "HX50", "CY700", "HY40"}) {
    CHECK_EQ_STR(run(c).c_str(), "ok");
  }
  CHECK(xAxis.microsteps == 16 && yAxis.microsteps == 4);
  CHECK(xAxis.runCurrent_mA == 600 && yAxis.runCurrent_mA == 700);
  CHECK_NEAR(xAxis.holdMultiplier, 0.50, 1e-6);
  CHECK_NEAR(yAxis.holdMultiplier, 0.40, 1e-6);
}

TEST(Stop_HaltsBothAxes) {
  resetFirmwareState();
  run("X800");
  run("Y800");
  CHECK_EQ_STR(run("!").c_str(), "ok");
  CHECK(!xAxis.stepper.isRunning() && !yAxis.stepper.isRunning());
}

TEST(JogSpeed_UsesFeed_ClampsIt_AndFallsBack) {
  resetFirmwareState();
  run("$J=G91G21X100F1000");
  CHECK_NEAR(xAxis.stepper.maxSpeed(), 1000, 0.001);
  run("$J=G53Y400F100");
  CHECK_NEAR(yAxis.stepper.maxSpeed(), 100, 0.001);
  run("$J=G91G21X100F10");
  CHECK_NEAR(xAxis.stepper.maxSpeed(), JOG_SPEED_MIN, 0.001);
  run("$J=G91G21X100F999999");
  CHECK_NEAR(xAxis.stepper.maxSpeed(), JOG_SPEED_MAX, 0.001);
  run("$J=G91G21X100");
  CHECK_NEAR(xAxis.stepper.maxSpeed(), DEFAULT_MAX_SPEED, 0.001);
  run("$J=G91G21X100F0");
  CHECK_NEAR(xAxis.stepper.maxSpeed(), DEFAULT_MAX_SPEED, 0.001);
}

TEST(AxisFirstLegacyCommands_AreAcknowledgedButChangeNothing) {
  resetFirmwareState();
  for (const char *c : {"XC600", "YC600", "XH50", "YH50"}) {
    CHECK_EQ_STR(run(c).c_str(), "ok");
  }
  CHECK(xAxis.runCurrent_mA == 600 && yAxis.runCurrent_mA == 600);
  CHECK_NEAR(xAxis.holdMultiplier, 0.25, 1e-6);
  CHECK(xAxis.stepper.targetPosition() == 0 && yAxis.stepper.targetPosition() == 0);
}

TEST(UnknownEmptyAndShortInputs_FollowWireDiscipline) {
  resetFirmwareState();
  CHECK_EQ_STR(run("definitely-not-a-command").c_str(), "ok");
  CHECK_EQ_STR(run("").c_str(), "");
  CHECK_EQ_STR(run("Z").c_str(), "error");
}

TEST(OnlyStatusProbeWritesToSerial_AmongNonHomingCommands) {
  // The web path discards replies; this proves the handlers it may reach
  // never print on their own.
  resetFirmwareState();
  for (const char *c : {"$J=G91G21X10F800", "$J=G53Y5F800", "X10", "Y-10", "!"}) run(c);
  CHECK_EQ_STR(Serial.out, "");
}
