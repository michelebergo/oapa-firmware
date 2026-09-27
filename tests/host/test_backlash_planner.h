// Port of BacklashModePlannerTest.cs, plus the valo_20260919 field plan.
#include "../../src/motion/backlash_planner.h"

namespace btest {

using motion::BacklashMode;
using motion::BacklashPlan;
using motion::BacklashPlanner;
using motion::Direction;

constexpr float B = 8.0f;    // configured backlash
constexpr float O = 2.5f;    // unidirectional overshoot margin: 0.25*B + 0.5'

inline BacklashPlan planSym(BacklashMode mode, float move, Direction last) {
  return BacklashPlanner::plan(mode, move, B, last);
}

inline int sgn(float v) { return v > 0 ? 1 : v < 0 ? -1 : 0; }

// Physical arrival of a plan on a mechanism that loses b on every reversal.
inline float arrive(const BacklashPlan &plan, float b, Direction last) {
  float position = 0;
  int lastSign = last == Direction::Positive ? 1 : -1;
  for (int i = 0; i < plan.count; ++i) {
    float move = plan.legs[i];
    int sign = sgn(move);
    float effective = std::fabs(move);
    if (sign != 0 && sign != lastSign) {
      effective = std::max(0.0f, effective - b);
      lastSign = sign;
    }
    position += sign * effective;
  }
  return position;
}

// Net displacement on a mechanism with per-direction lost motion.
inline float travel(const BacklashPlan &plan, float lostEnteringPositive, float lostEnteringNegative, Direction last) {
  float position = 0;
  int lastSign = last == Direction::Positive ? 1 : -1;
  for (int i = 0; i < plan.count; ++i) {
    float move = plan.legs[i];
    int sign = sgn(move);
    if (sign == 0) continue;
    float lost = sign != lastSign ? (sign > 0 ? lostEnteringPositive : lostEnteringNegative) : 0.0f;
    position += move - sign * std::min(std::fabs(move), lost);
    lastSign = sign;
  }
  return position;
}

inline bool isOne(const BacklashPlan &p, float a) { return p.count == 1 && std::fabs(p.legs[0] - a) < 1e-4f; }
inline bool isTwo(const BacklashPlan &p, float a, float b) {
  return p.count == 2 && std::fabs(p.legs[0] - a) < 1e-4f && std::fabs(p.legs[1] - b) < 1e-4f;
}
inline bool samePlan(const BacklashPlan &a, const BacklashPlan &b) {
  if (a.count != b.count) return false;
  for (int i = 0; i < a.count; ++i) {
    if (a.legs[i] != b.legs[i]) return false;
  }
  return true;
}

}  // namespace btest

TEST(Backlash_SameDirection_AllModes_PlainMove) {
  using namespace btest;
  for (auto mode : {BacklashMode::Off, BacklashMode::Soft, BacklashMode::Full, BacklashMode::Unidirectional}) {
    CHECK(isOne(planSym(mode, 10, Direction::Positive), 10));
  }
}

TEST(Backlash_Off_Reversal_PlainMove) {
  using namespace btest;
  CHECK(isOne(planSym(BacklashMode::Off, -10, Direction::Positive), -10));
}

TEST(Backlash_Full_Reversal_SingleMoveIncludesTheBacklash) {
  using namespace btest;
  auto plan = planSym(BacklashMode::Full, -10, Direction::Positive);
  CHECK(isOne(plan, -(10 + B)));
  CHECK_NEAR(arrive(plan, B, Direction::Positive), -10, 0.001);
}

TEST(Backlash_Soft_Reversal_SingleMoveIncludesThreeQuartersOfTheBacklash) {
  using namespace btest;
  auto plan = planSym(BacklashMode::Soft, -10, Direction::Positive);
  CHECK(isOne(plan, -(10 + 0.75f * B)));
  CHECK_NEAR(arrive(plan, B, Direction::Positive), -10 + 0.25f * B, 0.001);
}

TEST(Backlash_Unidirectional_Reversal_OvershootsAndReturnsFromThePreferredDirection) {
  using namespace btest;
  auto plan = planSym(BacklashMode::Unidirectional, -10, Direction::Positive);
  CHECK(isTwo(plan, -(10 + B + O), B + O));
  CHECK_NEAR(arrive(plan, B, Direction::Positive), -10, 0.001);
}

TEST(Backlash_Unidirectional_FinalApproachDirection_IsAlwaysPositive) {
  using namespace btest;
  auto plan = planSym(BacklashMode::Unidirectional, -10, Direction::Positive);
  CHECK(sgn(plan.legs[plan.count - 1]) == 1);
  CHECK_NEAR(arrive(plan, B, Direction::Positive), -10, 0.001);

  auto recovering = planSym(BacklashMode::Unidirectional, 10, Direction::Negative);
  CHECK(isOne(recovering, 10 + B));
  CHECK_NEAR(arrive(recovering, B, Direction::Negative), 10, 0.001);

  auto pinned = planSym(BacklashMode::Unidirectional, -10, Direction::Negative);
  CHECK(isTwo(pinned, -(10 + O), B + O));
  CHECK_NEAR(arrive(pinned, B, Direction::Negative), -10, 0.001);
}

TEST(Backlash_ZeroBacklash_ReversalIsAPlainMove_InEveryMode) {
  using namespace btest;
  for (auto mode : {BacklashMode::Soft, BacklashMode::Full, BacklashMode::Unidirectional}) {
    CHECK(isOne(BacklashPlanner::plan(mode, -10, 0.0f, Direction::Positive), -10));
  }
}

TEST(Backlash_Recommendation_FollowsTheMeasuredBacklashAndNoise) {
  using namespace btest;
  CHECK(BacklashPlanner::recommend(0.3f, 0.1f) == BacklashMode::Off);
  CHECK(BacklashPlanner::recommend(2.0f, 0.1f) == BacklashMode::Full);
  CHECK(BacklashPlanner::recommend(20.0f, 0.1f) == BacklashMode::Unidirectional);
  CHECK(BacklashPlanner::recommend(0.45f, 0.3f) == BacklashMode::Off);
}

TEST(Backlash_Unidirectional_WithDirectionalPlay_LandsExactly_WhenEachLegCarriesItsOwnValue) {
  using namespace btest;
  auto plan = BacklashPlanner::plan(BacklashMode::Unidirectional, -15, 16, 55, Direction::Positive);
  CHECK_NEAR(travel(plan, 16, 55, Direction::Positive), -15, 0.01);
}

TEST(Backlash_Unidirectional_WithDirectionalPlay_MissesByTheDifference_IfBothLegsShareOneValue) {
  using namespace btest;
  float mean = (16.0f + 55.0f) / 2.0f;
  auto plan = BacklashPlanner::plan(BacklashMode::Unidirectional, -15, mean, Direction::Positive);
  CHECK_NEAR(travel(plan, 16, 55, Direction::Positive), -15 + (55 - 16), 0.01);
}

TEST(Backlash_Full_WithDirectionalPlay_UsesTheDirectionTheMoveTravels) {
  using namespace btest;
  CHECK(isOne(BacklashPlanner::plan(BacklashMode::Full, -15, 16, 55, Direction::Positive), -70));
  CHECK(isOne(BacklashPlanner::plan(BacklashMode::Full, 15, 16, 55, Direction::Negative), 31));
}

TEST(Backlash_EqualValues_ReproduceTheSymmetricPlansExactly_InEveryMode) {
  using namespace btest;
  for (auto mode : {BacklashMode::Off, BacklashMode::Soft, BacklashMode::Full, BacklashMode::Unidirectional}) {
    for (float move : {-10.0f, 10.0f}) {
      Direction last = move > 0 ? Direction::Negative : Direction::Positive;
      CHECK(samePlan(BacklashPlanner::plan(mode, move, 6, 6, last), BacklashPlanner::plan(mode, move, 6.0f, last)));
    }
  }
}

TEST(Backlash_OneSidedPlay_TheFreeDirectionPaysNothing_ButTheArrivalStaysFromBelow) {
  using namespace btest;
  auto plan = BacklashPlanner::plan(BacklashMode::Unidirectional, -15, 40, 0, Direction::Positive);
  CHECK(isTwo(plan, -15 - 10.5f, 40 + 10.5f));
  CHECK_NEAR(travel(plan, 40, 0, Direction::Positive), -15, 0.01);
}

TEST(Backlash_Unidirectional_EveryCombinationOfSignAndEngagement_EndsTravellingPositive_AndArrivesExactly) {
  using namespace btest;
  const float pairs[4][2] = {{8, 8}, {16, 55}, {40, 0}, {0, 12}};
  for (float move : {-12.0f, -0.2f, 0.3f, 9.0f}) {
    for (auto last : {Direction::Positive, Direction::Negative}) {
      for (auto &pair : pairs) {
        auto plan = BacklashPlanner::plan(BacklashMode::Unidirectional, move, pair[0], pair[1], last);
        CHECK(sgn(plan.legs[plan.count - 1]) == 1);
        CHECK_NEAR(travel(plan, pair[0], pair[1], last), move, 0.01);
      }
    }
  }
}

TEST(Backlash_Recommendation_FollowsTheWorseDirection) {
  using namespace btest;
  CHECK(BacklashPlanner::recommend(1.0f, 20.0f, 0.1f) == BacklashMode::Unidirectional);
  CHECK(BacklashPlanner::recommend(0.2f, 2.0f, 0.1f) == BacklashMode::Full);
}

// valo_20260919 18:32:04, the line the plugin log prints itself:
//   move -0.29' planned as [-14.699338, 18.968803] (net 4.27')
// Same inputs as FieldReplayRegressionTest.ValoReal_20260919_TheSmallReversal_IsPlannedToArriveOnTheWrongSide:
// the port reproduces the field plan. On an axis with symmetric play (10-12' measured that
// night) the axis delivers the net, +4.27', not the request.
TEST(Backlash_ValoReal_20260919_TheSmallReversal_PlanMatchesTheFieldLog) {
  using namespace btest;
  auto plan = BacklashPlanner::plan(BacklashMode::Unidirectional, -0.29f, 14.78f, 10.21f, Direction::Positive, 0.25f);
  CHECK(plan.count == 2);
  CHECK_NEAR(plan.net(), 4.27, 0.01);
  for (float play : {10.0f, 11.0f, 12.0f}) {
    CHECK_NEAR(travel(plan, play, play, Direction::Positive), 4.27, 0.01);
  }
  auto withMean = BacklashPlanner::plan(BacklashMode::Unidirectional, -0.29f, 12.49f, 12.49f, Direction::Positive, 0.25f);
  CHECK_NEAR(travel(withMean, 11, 11, Direction::Positive), -0.29, 0.01);
}
