// Event log and error history: numbered ring buffers and their JSON.
#pragma once
#include <cstring>
#include <string>

TEST(LG_Ring_KeepsTheNewestNAndNumbersEveryEntry) {
  SeqRing<int, 3> ring;
  for (int i = 1; i <= 5; i++) CHECK(ring.push(i * 10) == static_cast<uint32_t>(i));
  SeqRing<int, 3>::Item out[5];
  size_t n = ring.since(0, out, 5);
  CHECK(n == 3 && ring.size() == 3 && ring.lastSeq() == 5);
  CHECK(out[0].seq == 3 && out[0].value == 30);
  CHECK(out[2].seq == 5 && out[2].value == 50);
}

TEST(LG_Ring_SincePagesOldestFirst) {
  SeqRing<int, 10> ring;
  for (int i = 1; i <= 6; i++) ring.push(i);
  SeqRing<int, 10>::Item out[2];
  CHECK(ring.since(2, out, 2) == 2);
  CHECK(out[0].seq == 3 && out[1].seq == 4);
  CHECK(ring.since(6, out, 2) == 0);
}

TEST(LG_MakeEvent_Truncates) {
  std::string longText(200, 'a');
  EventEntry e = makeEvent(42, "a-code-longer-than-fifteen", longText.c_str());
  CHECK(e.uptimeMs == 42);
  CHECK(std::strlen(e.code) == 15);
  CHECK(std::strlen(e.text) == 80);
}

TEST(LG_EventsJson_ExactBodyAndEscaping) {
  EventLog log;
  log.push(makeEvent(1500, "boot", "firmware 1.3.0"));
  log.push(makeEvent(2000, "wifi", "joined \"home\""));
  EventLog::Item items[4];
  size_t n = log.since(0, items, 4);
  char buf[512];
  int len = formatEventsJson(items, n, log.lastSeq(), buf, sizeof buf);
  CHECK_EQ_STR(buf, "{\"last\":2,\"events\":[{\"seq\":1,\"t\":1500,\"code\":\"boot\",\"text\":\"firmware 1.3.0\"},"
                    "{\"seq\":2,\"t\":2000,\"code\":\"wifi\",\"text\":\"joined \\\"home\\\"\"}]}");
  CHECK(len == static_cast<int>(std::strlen(buf)));
}

TEST(LG_HistoryJson_ExactBody) {
  HistoryLog log;
  HistorySample s;
  s.uptimeMs = 4000; s.azArcmin = 30.0f; s.altArcmin = -20.5f; s.moved = true;
  log.push(s);
  HistoryLog::Item items[2];
  size_t n = log.since(0, items, 2);
  char buf[128];
  formatHistoryJson(items, n, log.lastSeq(), buf, sizeof buf);
  CHECK_EQ_STR(buf, "{\"last\":1,\"samples\":[[1,4000,30.00,-20.50,1]]}");
}

TEST(LG_SettingsAndDriversJson_ExactBodies) {
  devset::LoopConfig c;
  c.factorX = 60; c.factorY = 60.5;
  char buf[256];
  formatSettingsJson(c, "192.168.1.53", buf, sizeof buf);
  CHECK_EQ_STR(buf, "{\"factorX\":60.000,\"factorY\":60.500,\"tolerance\":1.00,\"cap\":30.00,\"settleMs\":2000,\"feed\":1000,"
                    "\"calThresholdArcmin\":2.00,\"asiairHost\":\"192.168.1.53\"}");
  devset::DriverConfig x, y;
  y.runMa = 700;
  formatDriversJson(x, y, buf, sizeof buf);
  CHECK_EQ_STR(buf, "{\"x\":{\"run\":600,\"hold\":25,\"micro\":16},\"y\":{\"run\":700,\"hold\":25,\"micro\":16}}");
}

TEST(LG_Json_TooSmallBuffer_ReturnsMinusOne) {
  char buf[8];
  CHECK(formatSettingsJson(devset::LoopConfig(), "", buf, sizeof buf) == -1);
  EventLog log;
  log.push(makeEvent(1, "boot", "x"));
  EventLog::Item items[1];
  log.since(0, items, 1);
  CHECK(formatEventsJson(items, 1, 1, buf, sizeof buf) == -1);
}
