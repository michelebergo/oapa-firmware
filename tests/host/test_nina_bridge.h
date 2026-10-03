// N.I.N.A. as an error source: the serial grammar, the run-start rule, and the
// reading the loop must not trust after a move.
#include "../../src/nina_bridge.h"

using ninabridge::Kind;
using ninabridge::NinaSource;

TEST(NB_Reading_ParsesAzimuthAndAltitudeInArcmin) {
  auto c = ninabridge::parse("$E=-120.5,33.25");
  CHECK(c.kind == Kind::Reading);
  CHECK_NEAR(c.a, -120.5, 1e-9);
  CHECK_NEAR(c.b, 33.25, 1e-9);
}

TEST(NB_MalformedBridgeCommands_AreInvalid_NotSilentlyAccepted) {
  CHECK(ninabridge::parse("$E=1,2x").kind == Kind::Invalid);
  CHECK(ninabridge::parse("$E=1").kind == Kind::Invalid);
  CHECK(ninabridge::parse("$E=,2").kind == Kind::Invalid);
  CHECK(ninabridge::parse("$E=nan,2").kind == Kind::Invalid);
  CHECK(ninabridge::parse("$F=0,40").kind == Kind::Invalid);
  CHECK(ninabridge::parse("$A=2").kind == Kind::Invalid);
}

TEST(NB_FactorsStartStopAndStatus) {
  auto f = ninabridge::parse("$F=12.5,-40");
  CHECK(f.kind == Kind::Factors);
  CHECK_NEAR(f.a, 12.5, 1e-9);
  CHECK_NEAR(f.b, -40, 1e-9);
  CHECK(ninabridge::parse("$A=1").kind == Kind::Start);
  CHECK(ninabridge::parse("$A=0").kind == Kind::Stop);
  CHECK(ninabridge::parse("$L?").kind == Kind::StatusQuery);
}

TEST(NB_ThePluginGrammar_IsNotABridgeCommand) {
  for (const char *line : {"?", "!", "$H", "$J=G91G21X-280F1000", "X800", "CX600", "$L", "$E", "$X=1,2"}) {
    CHECK(ninabridge::parse(line).kind == Kind::None);
  }
}

TEST(NB_TheFirstReadingOfAStream_StartsARun_TheNextOnesDoNot) {
  NinaSource s;
  CHECK(s.onReading(10, 5, 1000, false));
  CHECK(!s.onReading(9, 5, 4000, true));
  CHECK(s.hasReading());
  CHECK_NEAR(s.latest().azArcmin, 9, 1e-9);
  CHECK(s.latest().receivedMs == 4000);
}

TEST(NB_ReadingsAfterARunEnded_DoNotRestartIt_UntilANewStream) {
  NinaSource s;
  s.onReading(10, 5, 0, false);
  // The run finished; TPPA keeps solving for a while.
  CHECK(!s.onReading(0.5, 0.2, 5000, false));
  CHECK(!s.onReading(0.6, 0.2, 5000 + NinaSource::kStreamGapMs, false));
  // Silence longer than the gap: the next reading is a new alignment.
  CHECK(s.onReading(20, 3, 5000 + 2 * NinaSource::kStreamGapMs + 1, false));
}

TEST(NB_ANewStream_WhileARunIsActive_DoesNotStartAnother) {
  NinaSource s;
  CHECK(!s.onReading(10, 5, 0, true));
}

// TPPA exposes and solves continuously, so the first reading that arrives after
// a move can come from an image exposed while the motors were turning. The
// loop set to skip one reading after each move must act on the next one.
TEST(LP_SkipAfterMotion_TheFirstReadingAfterAMoveIsNotActedOn) {
  LoopSettings s = lpSettings();
  s.readingsToSkipAfterMotion = 1;
  s.settleMs = 0;
  AlignmentLoop l(s);
  l.start(0, false);
  Observation first{30, -20, 100};
  CHECK(lpTick(l, 100, &first).kind == ActionKind::Move);
  lpTick(l, 150, nullptr, false, true);
  lpTick(l, 400, nullptr, true, true);  // X done, Y rounds to zero steps: back to waiting
  CHECK(l.phase() == Phase::WaitingObservation);

  Observation duringMotion{30, -20, 500};  // what an image exposed before the move still shows
  CHECK(lpTick(l, 500, &duringMotion).kind == ActionKind::None);
  CHECK(l.movesCommanded() == 1);

  Observation afterMotion{24.6, -20, 3000};
  CHECK(lpTick(l, 3000, &afterMotion).kind == ActionKind::Move);
  CHECK(l.movesCommanded() == 2);
}

TEST(LP_SkipAfterMotion_ZeroKeepsTheOriginalBehaviour) {
  LoopSettings s = lpSettings();
  s.settleMs = 0;
  AlignmentLoop l(s);
  l.start(0, false);
  Observation first{30, -20, 100};
  lpTick(l, 100, &first);
  lpTick(l, 150, nullptr, false, true);
  lpTick(l, 400, nullptr, true, true);
  Observation next{24.6, -20, 500};
  CHECK(lpTick(l, 500, &next).kind == ActionKind::Move);
  CHECK(l.movesCommanded() == 2);
}

TEST(NB_Status_IsOneParseableLine) {
  ninabridge::Status s;
  s.phase = "moving_x";
  s.source = "nina";
  s.moves = 3;
  s.hasReading = true;
  s.azArcmin = -2.1;
  s.altArcmin = 0.4;
  s.planX = -1.6;
  s.reason = "Correction | capped";
  char line[200];
  ninabridge::formatStatus(s, line, sizeof line);
  CHECK_EQ_STR(line, "<L|phase:moving_x|outcome:none|source:nina|moves:3|az:-2.10|alt:0.40|plan:-1.60,0.00|reason:Correction / capped|>");
}

TEST(NB_Status_WithoutAReading_SaysSo) {
  ninabridge::Status s;
  char line[200];
  ninabridge::formatStatus(s, line, sizeof line);
  CHECK_EQ_STR(line, "<L|phase:idle|outcome:none|source:none|moves:0|az:-|alt:-|plan:0.00,0.00|reason:|>");
}

// The TCP link rebuilds the "?" frame itself (the 1.2.2 handler writes to USB
// only); it has to be the same bytes, or the plugin's status regex would see
// two different devices depending on the link.
TEST(NB_TcpStatusFrame_IsByteIdenticalToTheUsbOne) {
  const long positions[][2] = {{0, 0}, {12, -3}, {-123456, 98765}};
  for (auto &p : positions) {
    resetFirmwareState();
    xAxis.stepper.setCurrentPosition(p[0]);
    yAxis.stepper.setCurrentPosition(p[1]);
    handleStatusQuery();
    char frame[96];
    ninabridge::formatStatusFrame("Idle", p[0], p[1], FW_VERSION, frame, sizeof frame);
    CHECK_EQ_STR(Serial.out, std::string(frame) + "\r\nok\r\n");
  }
  resetFirmwareState();
}

TEST(NB_Backlash_ParsesAxisModeAndTheTwoDirections) {
  auto c = ninabridge::parse("$B=X,U,14.78,10.21");
  CHECK(c.kind == Kind::Backlash);
  CHECK(c.axis == 'X' && c.mode == 'U');
  CHECK_NEAR(c.a, 14.78, 1e-9);
  CHECK_NEAR(c.b, 10.21, 1e-9);
  CHECK(ninabridge::parse("$B=Y,O,0,0").kind == Kind::Backlash);
}

TEST(NB_Backlash_Malformed_IsInvalid) {
  for (const char *line : {"$B=Z,U,1,1", "$B=X,Q,1,1", "$B=X,U,1", "$B=X,U,-1,1", "$B=XU,1,1", "$B=X,U,1,1x"}) {
    CHECK(ninabridge::parse(line).kind == Kind::Invalid);
  }
}

TEST(NB_Tolerance_ParsesArcmin_WithinBounds) {
  auto c = ninabridge::parse("$T=0.75");
  CHECK(c.kind == Kind::Tolerance);
  CHECK_NEAR(c.a, 0.75, 1e-9);
  for (const char *line : {"$T=0", "$T=-1", "$T=61", "$T=", "$T=1x"}) {
    CHECK(ninabridge::parse(line).kind == Kind::Invalid);
  }
}

TEST(NB_MoveCap_ParsesArcmin_WithinBounds) {
  auto c = ninabridge::parse("$M=180");
  CHECK(c.kind == Kind::MoveCap);
  CHECK_NEAR(c.a, 180.0, 1e-9);
  CHECK(ninabridge::parse("$M=1").kind == Kind::MoveCap);
  for (const char *line : {"$M=0", "$M=0.5", "$M=-5", "$M=181", "$M=", "$M=30x"}) {
    CHECK(ninabridge::parse(line).kind == Kind::Invalid);
  }
}

TEST(NB_MoveCapQuery_ReportsTheLargestMoveCapTheBoardAccepts) {
  // The host reads the limit instead of assuming it, so raising it needs no host change.
  // Firmware before 1.3.2 does not know $M? and acknowledges it with "ok": the host takes 120.
  CHECK(ninabridge::parse("$M?").kind == Kind::MoveCapQuery);
  char line[64];
  ninabridge::formatMoveCapLimit(line, sizeof line);
  CHECK_EQ_STR(line, "<M|max:180|>");
}

TEST(NB_Calibration_StartStopAndQuery) {
  CHECK(ninabridge::parse("$C=1").kind == Kind::CalibrateStart);
  CHECK(ninabridge::parse("$C=0").kind == Kind::CalibrateStop);
  CHECK(ninabridge::parse("$C=2").kind == Kind::CalibrateOnly);
  CHECK(ninabridge::parse("$C=3").kind == Kind::Invalid);
  CHECK(ninabridge::parse("$K?").kind == Kind::CalibrationQuery);
}

TEST(NB_CalibrationStatus_IsOneParseableLine) {
  ninabridge::CalibrationStatus s;
  s.state = "done";
  s.xValid = true;
  s.xFactor = 12.346;
  s.xSign = 1;
  s.yValid = true;
  s.yFactor = 40.1;
  s.ySign = -1;
  s.xPlayValid = true;
  s.xPlayPositive = 4.2;
  s.xPlayNegative = 4.35;
  s.xMode = 'U';
  s.reason = "Calibration complete";
  char line[200];
  ninabridge::formatCalibration(s, line, sizeof line);
  CHECK_EQ_STR(line, "<K|state:done|x:12.35,+1|y:40.10,-1|xplay:4.20,4.35,U|yplay:-|reason:Calibration complete|>");

  ninabridge::CalibrationStatus none;
  ninabridge::formatCalibration(none, line, sizeof line);
  CHECK_EQ_STR(line, "<K|state:idle|x:-|y:-|xplay:-|yplay:-|reason:|>");
}

TEST(NB_Status_CarriesTheWholeReason_ThePluginLogsIt) {
  // valo_20260928: the NINA log cut the halt reason at "calibration factors or",
  // leaving out the backlash compensation that was the cause.
  ninabridge::Status s;
  s.phase = "ended";
  s.outcome = "halted_estimate_drift";
  s.source = "nina";
  s.moves = 10;
  s.reason =
      "Error increased for 3 consecutive measurements while corrections were small; the error estimate appears to "
      "have drifted. This is not a calibration problem - re-run the alignment to re-measure.";
  char line[400];
  ninabridge::formatStatus(s, line, sizeof line);
  std::string text(line);
  CHECK(text.find("re-run the alignment to re-measure.|>") != std::string::npos);

  ninabridge::CalibrationStatus c;
  c.reason = s.reason;
  ninabridge::formatCalibration(c, line, sizeof line);
  text = line;
  CHECK(text.find("re-run the alignment to re-measure.|>") != std::string::npos);
}

TEST(NB_EventQuery_AsksForTheEventAfterASequenceNumber) {
  auto q = ninabridge::parse("$G=0");
  CHECK(q.kind == Kind::EventQuery);
  CHECK(q.seq == 0);
  q = ninabridge::parse("$G=4294967295");
  CHECK(q.kind == Kind::EventQuery);
  CHECK(q.seq == 4294967295u);
  for (const char *line : {"$G=", "$G=-1", "$G=1.5", "$G=x", "$G=12 3", "$G=4294967296"}) {
    CHECK(ninabridge::parse(line).kind == Kind::Invalid);
  }
}

TEST(NB_Event_IsOneParseableLine_WithTheNewestSequenceNumber) {
  // The plugin pages through the board's event log one event per command, and
  // starts over when "last" is below what it asked for: the board restarted.
  EventLog::Item item;
  item.seq = 5;
  item.value = makeEvent(123456, "move", "correction AZ 0.28' | ALT 0.07'");
  char line[200];
  ninabridge::formatEvent(&item, 7, line, sizeof line);
  CHECK_EQ_STR(line, "<G|seq:5|last:7|ms:123456|code:move|text:correction AZ 0.28' / ALT 0.07'|>");

  ninabridge::formatEvent(nullptr, 7, line, sizeof line);
  CHECK_EQ_STR(line, "<G|seq:-|last:7|>");
}
