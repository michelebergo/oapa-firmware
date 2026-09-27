// /api/status body. Strings are firmware constants (no escaping needed).
#pragma once

static StatusSnapshot sampleSnapshot() {
  StatusSnapshot s;
  s.x = 1234; s.y = -56; s.running = true; s.homing = false; s.ninaActive = true;
  s.stationConnected = true; s.rssi = -48; s.uptimeS = 12; s.bootCount = 3;
  s.resetReason = "POWERON"; s.fwVersion = "1.3.0";
  return s;
}

TEST(StatusJson_ExactBody) {
  char buf[256];
  int n = formatStatusJson(sampleSnapshot(), buf, sizeof buf);
  CHECK_EQ_STR(buf, "{\"x\":1234,\"y\":-56,\"state\":\"Run\",\"ninaActive\":true,"
                    "\"network\":\"station\",\"rssi\":-48,\"uptime\":12,\"boots\":3,"
                    "\"reset\":\"POWERON\",\"fw\":\"1.3.0\"}");
  CHECK(n == (int)std::strlen(buf));
}

TEST(StatusJson_HomeWinsOverRun_AndSetupNetwork) {
  StatusSnapshot s = sampleSnapshot();
  s.homing = true;
  s.stationConnected = false;
  char buf[256];
  formatStatusJson(s, buf, sizeof buf);
  CHECK(std::strstr(buf, "\"state\":\"Home\"") != nullptr);
  CHECK(std::strstr(buf, "\"network\":\"setup\"") != nullptr);
}

TEST(StatusJson_IdleWhenStill) {
  StatusSnapshot s = sampleSnapshot();
  s.running = false;
  char buf[256];
  formatStatusJson(s, buf, sizeof buf);
  CHECK(std::strstr(buf, "\"state\":\"Idle\"") != nullptr);
}

TEST(StatusJson_TooSmallBuffer_ReturnsMinusOne) {
  char buf[16];
  CHECK(formatStatusJson(sampleSnapshot(), buf, sizeof buf) == -1);
}
