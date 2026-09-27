// Port of BacklashModePlanner.cs (TPPA plugin, merged in #23): pure move
// planning for the four OAPA backlash modes and the mode recommendation rule.
// Constants and plan shapes follow the C# original; see its comments for the
// field cases behind them. Arithmetic is float, as in the original.
#pragma once

namespace motion {

enum class BacklashMode { Off, Soft, Full, Unidirectional };
enum class Direction { Negative, Positive };

// At most two legs: an overshoot and a return.
struct BacklashPlan {
  int count = 0;
  float legs[2] = {0, 0};

  float net() const { return count == 2 ? legs[0] + legs[1] : count == 1 ? legs[0] : 0.0f; }
};

class BacklashPlanner {
 public:
  static constexpr float kSoftFraction = 0.75f;
  static constexpr float kOvershootFractionOfBacklash = 0.25f;
  static constexpr float kOvershootFloorArcmin = 0.5f;
  static constexpr float kMeasurableSigmaFactor = 2.0f;
  static constexpr float kMeasurableFloorArcmin = 0.5f;
  static constexpr float kLargeBacklashArcmin = 3.0f;

  // Plans a relative move on an axis whose play costs the same in both directions.
  static BacklashPlan plan(BacklashMode mode, float move, float backlashArcmin, Direction last);

  // Plans a relative move with per-direction play. An empty plan means the
  // reversal is finer than minimumReversalArcmin (zero disables the check).
  static BacklashPlan plan(BacklashMode mode, float move, float backlashEnteringPositive,
                           float backlashEnteringNegative, Direction last, float minimumReversalArcmin = 0.0f);

  static BacklashMode recommend(float backlashArcmin, float noiseSigmaArcmin);
  static BacklashMode recommend(float backlashEnteringPositive, float backlashEnteringNegative, float noiseSigmaArcmin);
};

}  // namespace motion
