// N.I.N.A. as an error source: the TPPA plugin forwards every polar error it
// measures and the firmware runs the alignment itself. Pure, host-tested.
//
// Serial commands (one reply line each, like every non-"?" command):
//   $E=<az>,<alt>   error reading in arcmin, TPPA signs         -> ok
//   $F=<fx>,<fy>    steps per azimuth / altitude arcminute       -> ok
//   $A=1 / $A=0     start a run now / stop the run               -> ok
//   $B=<axis>,<mode>,<plus>,<minus>  backlash of one axis for the loop's moves:
//                   axis X|Y, mode O(ff)|S(oft)|F(ull)|U(nidirectional), play in
//                   arcmin entering the positive / negative direction -> ok
//   $T=<arcmin>     alignment tolerance of runs N.I.N.A. feeds (TPPA's own,
//                   so both stop at the same error), 0 < t <= 60   -> ok
//   $M=<arcmin>     largest single correction of runs N.I.N.A. feeds, 1-180;
//                   until sent, the move cap saved on the board applies -> ok
//   $C=1 / $C=0     calibrate before the next alignment / stop a calibration -> ok
//   $C=2            calibrate only, no alignment after: the readings are field
//                   displacements the plugin measures itself, not a polar error -> ok
//   $K?             calibration state and result, one line       -> <K|...|>
//   $L?             loop status, one line                        -> <L|...|>
//   $M?             the largest move cap $M= accepts             -> <M|max:180|>
//   $G=<seq>        the board's event log, one event per command: the oldest
//                   event after seq, and the newest sequence number
//                   -> <G|seq:5|last:7|ms:123456|code:move|text:...|>, or
//                   <G|seq:-|last:7|> when nothing is newer. "last" below the
//                   seq asked for means the board restarted: ask from 0 again.
// A malformed bridge command replies "error". Anything else is not a bridge
// command and goes to the 1.2.2 dispatcher unchanged.
#pragma once
#include <cstddef>
#include <cstdint>

#include "device_logs.h"
#include "loop/observation.h"

namespace ninabridge {

enum class Kind { None, Reading, Factors, Start, Stop, StatusQuery, Backlash, Tolerance, MoveCap, CalibrateStart, CalibrateOnly, CalibrateStop, CalibrationQuery, EventQuery, MoveCapQuery, Invalid };

struct Command {
  Kind kind = Kind::None;
  double a = 0;  // az arcmin, or X factor
  double b = 0;  // alt arcmin, or Y factor
  char axis = 0;  // Backlash: 'X' or 'Y'
  char mode = 0;  // Backlash: 'O', 'S', 'F' or 'U'
  uint32_t seq = 0;  // EventQuery: the event after this sequence number
};

Command parse(const char *line);

// What $L? reports, one line:
//   <L|phase:moving_x|outcome:none|source:nina|moves:3|az:-2.10|alt:0.40|plan:-1.60,0.00|reason:...|>
struct Status {
  const char *phase = "idle";
  const char *outcome = "none";
  const char *source = "none";
  int moves = 0;
  bool hasReading = false;
  double azArcmin = 0;
  double altArcmin = 0;
  double planX = 0;
  double planY = 0;
  const char *reason = "";
};

// The longest reason a status line carries whole: the plugin logs it, and a
// halt's cause is at its end.
constexpr size_t kReasonMax = 256;

// Writes the line (no newline); '|' in the reason becomes '/' so the frame stays parseable.
size_t formatStatus(const Status &status, char *out, size_t len);

// The "?" status frame exactly as the 1.2.2 handler prints it on USB, for the
// TCP link, where that handler cannot write: <Idle|MPos:12.00,-3.00,0.00|V:1.6.0|>
size_t formatStatusFrame(const char *state, long x, long y, const char *version, char *out, size_t len);

// What $K? reports, one line; an axis without a result shows "-". The play is
// arcmin entering the positive, then the negative direction, then the backlash
// mode the loop uses with their mean (O/S/F/U):
//   <K|state:done|x:12.34,+1|y:40.10,-1|xplay:4.20,4.35,U|yplay:0.60,0.55,F|reason:Calibration complete|>
struct CalibrationStatus {
  const char *state = "idle";
  bool xValid = false;
  double xFactor = 0;
  int xSign = 0;
  bool yValid = false;
  double yFactor = 0;
  int ySign = 0;
  bool xPlayValid = false;
  double xPlayPositive = 0;
  double xPlayNegative = 0;
  char xMode = 'O';
  bool yPlayValid = false;
  double yPlayPositive = 0;
  double yPlayNegative = 0;
  char yMode = 'O';
  const char *reason = "";
};

size_t formatCalibration(const CalibrationStatus &status, char *out, size_t len);

// What $M? reports: the largest move cap $M= accepts (move_cap.h).
size_t formatMoveCapLimit(char *out, size_t len);

// What $G= reports; a null item means nothing newer than the seq asked for.
size_t formatEvent(const EventLog::Item *item, uint32_t lastSeq, char *out, size_t len);

// Decides when a stream of readings starts a run. TPPA solves every few
// seconds while it runs, so a reading after a silence longer than
// kStreamGapMs is a new alignment; readings that keep coming after a run
// ended (finished, halted, stopped) do not restart it.
class NinaSource {
 public:
  static constexpr uint32_t kStreamGapMs = 30000;

  // Records the reading; true when it should start a new run.
  bool onReading(double azArcmin, double altArcmin, uint32_t nowMs, bool runActive);

  bool hasReading() const { return hasReading_; }
  const paloop::Observation &latest() const { return latest_; }
  bool streaming(uint32_t nowMs) const { return hasReading_ && nowMs - latest_.receivedMs <= kStreamGapMs; }

 private:
  paloop::Observation latest_;
  bool hasReading_ = false;
};

}  // namespace ninabridge
