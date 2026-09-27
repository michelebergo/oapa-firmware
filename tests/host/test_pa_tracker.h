// Block C spec 3.1: which ASIAIR readings the alignment may use and when the PA ended.
#pragma once
#include <cmath>
#include <cstring>
using asiair::PaTracker;

static asiair::AsiairEvent tkPa(const char *state, bool error = false, double x = 0, double y = 0, int retry = 1) {
  asiair::AsiairEvent e;
  e.kind = asiair::EventKind::Pa3ppa;
  std::strcpy(e.state, state);
  e.hasError = error;
  e.xDeg = x;
  e.yDeg = y;
  e.retryCnt = retry;
  return e;
}

static asiair::AsiairEvent tkExposureStart() {
  asiair::AsiairEvent e;
  e.kind = asiair::EventKind::PaExposure;
  std::strcpy(e.state, "start");
  return e;
}

static asiair::AsiairEvent tkSolveFail(int code) {
  asiair::AsiairEvent e;
  e.kind = asiair::EventKind::PaSolve;
  std::strcpy(e.state, "fail");
  e.code = code;
  return e;
}

TEST(PT_ReadingIsTimedAtItsExposureStart) {
  PaTracker t;
  t.onEvent(tkPa("start"), 0);
  CHECK(!t.adjusting() && !t.hasReading());
  t.onEvent(tkExposureStart(), 1000);
  t.onEvent(tkPa("calc4", true, -1.0, 0.5), 4000);
  CHECK(t.adjusting() && t.hasReading());
  CHECK_NEAR(t.reading().azArcmin, -60, 1e-9);
  CHECK_NEAR(t.reading().altArcmin, 30, 1e-9);
  CHECK(t.reading().receivedMs == 1000);
  CHECK(t.readingArrivedMs() == 4000);
  CHECK_EQ_STR(t.phase(), "calc4");
}

TEST(PT_WithoutAnExposureStart_TheArrivalTimeIsUsed) {
  PaTracker t;
  t.onEvent(tkPa("calc3", true, 1.0, 1.0), 7000);
  CHECK(t.reading().receivedMs == 7000);
}

TEST(PT_FramesAfterAFailedSolveAreRejected) {
  PaTracker t;
  t.onEvent(tkExposureStart(), 1000);
  t.onEvent(tkPa("calc4", true, -1.0, 0.5), 4000);
  t.onEvent(tkSolveFail(251), 6000);
  CHECK(!t.ended());
  t.onEvent(tkExposureStart(), 7000);
  t.onEvent(tkPa("calc4", true, 5.0, 5.0, 2), 9000);
  CHECK(t.rejected() == 1);
  CHECK(t.adjusting());
  CHECK_NEAR(t.reading().azArcmin, -60, 1e-9);
  CHECK(t.reading().receivedMs == 1000);
}

// Field 16/09/2026: ASIAIR aborted a solve (code 253) twice in the middle of a live
// PA and was back in calc4 nine seconds later. An aborted solve says nothing about
// the PA being over; only silence does.
TEST(PT_AnAbortedSolveDoesNotEndAPaThatKeepsGoing) {
  PaTracker t;
  t.onEvent(tkExposureStart(), 3000);
  t.onEvent(tkPa("calc4", true, 1.0, 1.0), 4000);
  t.onEvent(tkSolveFail(253), 5000);
  CHECK(!t.ended());
  CHECK(t.adjusting() && t.hasReading());
  t.onEvent(tkExposureStart(), 13000);
  t.onEvent(tkPa("calc4", true, 0.9, 0.8), 14000);
  CHECK(!t.ended() && t.adjusting());
  CHECK_NEAR(t.reading().azArcmin, 54, 1e-9);
  t.tick(43999);  // the user really closes the PA: the results stop
  CHECK(!t.ended());
  t.tick(44000);
  CHECK(t.ended());
  CHECK_EQ_STR(t.endReason(), "no PA result from ASIAIR for 30 s");
}

// The link itself: ASIAIR stopped feeding an open socket at 00:49 on 16/09/2026,
// without closing it, and the board stayed on that dead connection for 23 minutes.
TEST(PT_LinkIsStaleAfterThirtySecondsWithoutData) {
  CHECK(!asiair::linkStale(1000, 30999));
  CHECK(asiair::linkStale(1000, 31000));
  CHECK(asiair::linkStale(0xFFFFF000u, 0xFFFFF000u + 30000u));
  CHECK(!asiair::linkStale(0xFFFFF000u, 0x00001000u));  // 8 s across the millis() wrap
}

TEST(PT_ThirtySecondsWithoutAResultEndsThePa) {
  PaTracker t;
  t.onEvent(tkPa("calc4", true, 1.0, 1.0), 4000);
  t.tick(33999);
  CHECK(!t.ended());
  t.tick(34000);
  CHECK(t.ended());
  CHECK_EQ_STR(t.endReason(), "no PA result from ASIAIR for 30 s");
}

TEST(PT_ANewStartClearsTheEnd) {
  PaTracker t;
  t.onEvent(tkPa("calc4", true, 1.0, 1.0), 4000);
  t.tick(34000);  // the previous PA ended in silence
  CHECK(t.ended());
  t.onEvent(tkPa("start"), 40000);
  CHECK(!t.ended() && !t.hasReading());
  t.onEvent(tkExposureStart(), 50000);
  t.onEvent(tkPa("calc3", true, 2.0, 2.0), 52000);
  CHECK(t.hasReading() && t.reading().receivedMs == 50000);
}

// Spec 8: a frame whose exposure began before move end + settle is never consumed,
// even though its result arrives after the settle.
TEST(PT_WithTheLoop_AFrameExposedBeforeTheSettleIsNeverUsed) {
  paloop::LoopSettings s;
  s.factorX = 60;
  s.factorY = 60;  // settle 2000 ms
  paloop::AlignmentLoop loop(s);
  PaTracker t;
  CHECK(loop.start(0, false));
  t.onEvent(tkExposureStart(), 100);
  t.onEvent(tkPa("calc4", true, 0.5, -0.3), 3000);  // 30' / -18'
  paloop::TickInput in;
  in.nowMs = 3000;
  in.observation = &t.reading();
  CHECK(loop.tick(in).kind == paloop::ActionKind::Move);
  in.nowMs = 3050;
  in.axisIdle[0] = false;
  loop.tick(in);
  in.axisIdle[0] = true;
  for (uint32_t now = 3100; now <= 3400; now += 50) {  // X completes; a Y move, if any, completes on the grace
    in.nowMs = now;
    loop.tick(in);
  }
  CHECK(loop.phase() == paloop::Phase::WaitingObservation);  // settle ends by 5400 at the latest

  t.onEvent(tkExposureStart(), 5000);
  t.onEvent(tkPa("calc4", true, 0.4, -0.3), 7000);  // exposed before the settle ended
  in.nowMs = 7000;
  in.observation = &t.reading();
  loop.tick(in);
  CHECK_NEAR(loop.lastTotalErrorArcmin(), std::hypot(30.0, 18.0), 1e-9);

  t.onEvent(tkExposureStart(), 7000);
  t.onEvent(tkPa("calc4", true, 0.4, -0.3), 10000);
  in.nowMs = 10000;
  in.observation = &t.reading();
  loop.tick(in);
  CHECK_NEAR(loop.lastTotalErrorArcmin(), std::hypot(24.0, 18.0), 1e-9);
}
