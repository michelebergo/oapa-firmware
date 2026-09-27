// /api/loop body. The reason comes from the loop and is escaped for JSON.
#pragma once
#include <cstring>

static LoopStatusSnapshot lsSample() {
  LoopStatusSnapshot s;
  std::strcpy(s.phase, "moving_x");
  std::strcpy(s.outcome, "none");
  std::strcpy(s.reason, "Probing azimuth response");
  s.simulation = true;
  std::strcpy(s.source, "sim");
  s.sourceReady = true;
  s.hasReading = true;
  s.azArcmin = 30.0;
  s.altArcmin = -20.0;
  s.totalArcmin = 36.0555;
  s.moves = 1;
  s.planX = 5.408;
  s.planY = 0;
  s.planProbe = true;
  s.capArcmin = 28.84;
  s.passMaxUs = 57;
  return s;
}

TEST(LS_ExactBody) {
  char buf[512];
  int n = formatLoopJson(lsSample(), buf, sizeof buf);
  CHECK_EQ_STR(buf, "{\"phase\":\"moving_x\",\"outcome\":\"none\",\"reason\":\"Probing azimuth response\","
                    "\"simulation\":true,\"source\":\"sim\",\"sourceReady\":true,\"reading\":true,\"az\":30.00,"
                    "\"alt\":-20.00,\"total\":36.06,\"moves\":1,"
                    "\"plan\":{\"x\":5.408,\"y\":0.000,\"probe\":true},\"cap\":28.84,\"passMaxUs\":57}");
  CHECK(n == static_cast<int>(std::strlen(buf)));
}

TEST(LS_ReasonIsEscaped) {
  LoopStatusSnapshot s = lsSample();
  std::strcpy(s.reason, "say \"hi\"\\ok\nnext");
  char buf[512];
  formatLoopJson(s, buf, sizeof buf);
  CHECK(std::strstr(buf, "\"reason\":\"say \\\"hi\\\"\\\\ok\\u000anext\"") != nullptr);
}

TEST(LS_TooSmallBuffer_ReturnsMinusOne) {
  char buf[32];
  CHECK(formatLoopJson(lsSample(), buf, sizeof buf) == -1);
}
