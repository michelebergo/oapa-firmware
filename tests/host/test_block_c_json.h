// /api/asiair and /api/calibrate bodies.
#pragma once
#include <cstring>

TEST(BC_AsiairJson_ExactBody) {
  AsiairSnapshot s;
  s.connected = true;
  std::strcpy(s.host, "10.0.0.1");
  std::strcpy(s.phase, "calc4");
  s.adjusting = true;
  s.hasReading = true;
  s.readingAgeMs = 3000;
  s.azArcmin = -4574.47;
  s.altArcmin = 5691.68;
  s.lines = 120;
  s.badLines = 1;
  s.rejected = 2;
  char buf[512];
  int n = formatAsiairJson(s, buf, sizeof buf);
  CHECK_EQ_STR(buf, "{\"connected\":true,\"host\":\"10.0.0.1\",\"phase\":\"calc4\",\"adjusting\":true,\"ended\":false,"
                    "\"endReason\":\"\",\"reading\":true,\"ageMs\":3000,\"az\":-4574.47,\"alt\":5691.68,"
                    "\"lines\":120,\"badLines\":1,\"rejected\":2}");
  CHECK(n == static_cast<int>(std::strlen(buf)));
}

TEST(BC_CalibrationJson_ExactBody) {
  CalibrationSnapshot s;
  std::strcpy(s.state, "done");
  s.axis = 'Y';
  s.probe = 2;
  s.steps = 600;
  s.responseArcmin = -3.33;
  std::strcpy(s.reason, "Calibration complete");
  s.xValid = true;
  s.xFactor = 180.25;
  s.xSign = 1;
  s.yValid = true;
  s.yFactor = 179.5;
  s.ySign = -1;
  s.offsetX = 0;
  s.offsetY = -12;
  char buf[512];
  formatCalibrationJson(s, buf, sizeof buf);
  CHECK_EQ_STR(buf, "{\"state\":\"done\",\"axis\":\"Y\",\"probe\":2,\"steps\":600,\"response\":-3.33,"
                    "\"reason\":\"Calibration complete\",\"x\":{\"valid\":true,\"factor\":180.25,\"sign\":1},"
                    "\"y\":{\"valid\":true,\"factor\":179.50,\"sign\":-1},\"offset\":{\"x\":0,\"y\":-12}}");
}

TEST(BC_Json_TooSmallBuffer_ReturnsMinusOne) {
  char buf[16];
  CHECK(formatAsiairJson(AsiairSnapshot(), buf, sizeof buf) == -1);
  CHECK(formatCalibrationJson(CalibrationSnapshot(), buf, sizeof buf) == -1);
}
