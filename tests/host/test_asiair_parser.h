// ASIAIR event lines, verbatim from captures/pa_night_20260913.log (13/09/2026).
#pragma once
#include <cstring>
#include <string>
using asiair::AsiairEvent;
using asiair::EventKind;

static const char *kCalc3 = R"J({"Event":"3PPA","Timestamp":"534.485105319","state":"calc3","state_code":14,"auto_move":true,"auto_update":false,"paused":false,"detail":{},"pa_error":{"total":121.714662,"x":-75.941030,"y":95.117921,"total_arcsec":438172.784795,"x_arcsec":-273387.708975,"y_arcsec":342424.517107},"lapse_ms":28176,"retry_cnt":1})J";
static const char *kMove1 = R"J({"Event":"3PPA","Timestamp":"510.108653696","state":"move1","state_code":5,"auto_move":true,"auto_update":false,"paused":false,"detail":{"mount_move_ok":false},"lapse_ms":3801,"retry_cnt":0})J";
static const char *kRetry2 = R"J({"Event":"3PPA","Timestamp":"925.457842613","state":"calc4","state_code":17,"auto_move":true,"auto_update":true,"paused":false,"detail":{},"pa_error":{"total":102.820961,"x":-40.696631,"y":94.424224,"total_arcsec":370155.458619,"x_arcsec":-146507.872953,"y_arcsec":339927.207955},"lapse_ms":419148,"retry_cnt":2})J";
static const char *kExposureStart = R"J({"Event":"Exposure","Timestamp":"286.376193006","page":"pa","tag":"3PPA","pa_state":"exp1","pa_state_code":2,"state":"start","exp_us":1000000,"gain":28})J";
static const char *kSolveFail = R"J({"Event":"PlateSolve","Timestamp":"295.158319367","page":"pa","tag":"3PPA","pa_state":"solve1","pa_state_code":3,"error":"solve failed","code":251,"state":"fail","result":{"star_number":24,"duration_ms":7133}})J";
static const char *kAborted = R"J({"Event":"PlateSolve","Timestamp":"306.185965974","page":"pa","tag":"3PPA","pa_state":"solve1","pa_state_code":3,"error":"aborted","code":253,"state":"fail","result":{"duration_ms":1144}})J";
static const char *kPiStatus = R"J({"Event":"PiStatus","Timestamp":"99.719519513","is_overtemp":false,"temp":37.900002,"is_undervolt":false,"is_over_current":false})J";
static const char *kPreviewExposure = R"J({"Event":"Exposure","Timestamp":"189.999420617","page":"preview","state":"start","exp_us":3000000,"gain":28})J";

TEST(AP_Calc3_YieldsTheErrorInDegrees) {
  AsiairEvent e;
  CHECK(asiair::parseLine(kCalc3, e));
  CHECK(e.kind == EventKind::Pa3ppa);
  CHECK_EQ_STR(e.state, "calc3");
  CHECK(e.stateCode == 14);
  CHECK(e.hasError);
  CHECK_NEAR(e.xDeg, -75.941030, 1e-9);
  CHECK_NEAR(e.yDeg, 95.117921, 1e-9);
  CHECK(e.retryCnt == 1);
  CHECK(e.code == 0);
}

TEST(AP_Move1_HasNoError) {
  AsiairEvent e;
  CHECK(asiair::parseLine(kMove1, e));
  CHECK(e.kind == EventKind::Pa3ppa);
  CHECK_EQ_STR(e.state, "move1");
  CHECK(e.stateCode == 5);
  CHECK(!e.hasError);
  CHECK(e.retryCnt == 0);
}

TEST(AP_FrameAfterAFailedSolve_CarriesRetryTwo) {
  AsiairEvent e;
  CHECK(asiair::parseLine(kRetry2, e));
  CHECK(e.retryCnt == 2);
  CHECK_NEAR(e.xDeg, -40.696631, 1e-9);
}

TEST(AP_PaExposureStart_AndSolveCodes) {
  AsiairEvent e;
  CHECK(asiair::parseLine(kExposureStart, e));
  CHECK(e.kind == EventKind::PaExposure);
  CHECK_EQ_STR(e.state, "start");
  CHECK(asiair::parseLine(kSolveFail, e));
  CHECK(e.kind == EventKind::PaSolve);
  CHECK_EQ_STR(e.state, "fail");
  CHECK(e.code == 251);
  CHECK(asiair::parseLine(kAborted, e));
  CHECK(e.code == 253);
}

TEST(AP_EventsOutsideThePa_AreOther) {
  AsiairEvent e;
  CHECK(asiair::parseLine(kPiStatus, e));
  CHECK(e.kind == EventKind::Other);
  CHECK(asiair::parseLine(kPreviewExposure, e));
  CHECK(e.kind == EventKind::Other);
}

TEST(AP_RejectsTruncatedAndNonObjectLines_AcceptsCrLf) {
  AsiairEvent e;
  std::string truncated(kCalc3, std::strlen(kCalc3) - 20);
  CHECK(!asiair::parseLine(truncated.c_str(), e));
  CHECK(!asiair::parseLine("", e));
  CHECK(!asiair::parseLine("hello", e));
  CHECK(!asiair::parseLine("{\"Timestamp\":\"1\"}", e));
  CHECK(!asiair::parseLine(nullptr, e));
  std::string crlf = std::string(kCalc3) + "\r\n";
  CHECK(asiair::parseLine(crlf.c_str(), e));
  CHECK(e.hasError);
}
