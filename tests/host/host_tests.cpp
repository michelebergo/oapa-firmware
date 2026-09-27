// Single translation unit that owns the firmware globals: the protocol region
// defines objects, so it is included exactly once, here.
#include "stubs/Arduino.h"
#include "stubs/AccelStepper.h"
#include "stubs/TMCStepper.h"

#define FW_VERSION "1.3.0"
#define DRIVER_TMC2209 1  // the E4 build; main.cpp sets it for the firmware
#include "../../src/oapa_protocol.inc"
#include "../../src/web_bridge.h"
#include "../../src/web_dispatch.inc"
#include "../../src/json_util.h"
#include "../../src/device_config.h"
#include "../../src/device_logs.h"
#include "../../src/page_json.h"
#include "../../src/status_json.h"
#include "../../src/net_rules.h"
#include "../../src/loop/convergence_monitor.h"
#include "../../src/loop/adjust_controller.h"
#include "../../src/loop/platform_sim.h"
#include "../../src/loop/alignment_loop.h"
#include "../../src/loop/calibration.h"
#include "../../src/loop/loop_status_json.h"
#include "../../src/asiair/asiair_parser.h"
#include "../../src/asiair/pa_tracker.h"
#include "../../src/asiair/asiair_status.h"
#include "../../src/loop/calibration_status.h"
#include "../../src/ota_rules.h"
#include "../../src/motion/backlash_planner.h"
#include "../../src/nina_bridge.h"

#include "tinytest.h"

// Restores the power-on state of 1.2.2 between tests.
void resetFirmwareState() {
  Serial.out.clear();
  Serial.in.clear();
  for (Axis *axis : {&xAxis, &yAxis}) {
    axis->stepper.setCurrentPosition(0);
    axis->stepper.setMaxSpeed(DEFAULT_MAX_SPEED);
    axis->runCurrent_mA = 600;
    axis->holdMultiplier = 0.25f;
    axis->microsteps = 16;
  }
  homingInProgress = false;
  lineBuffer = "";
}

#include "test_dispatch.h"
#include "test_web_bridge.h"
#include "test_status_json.h"
#include "test_net_rules.h"
#include "test_convergence_monitor.h"
#include "test_adjust_controller.h"
#include "test_field_replay.h"
#include "test_platform_sim.h"
#include "test_alignment_loop.h"
#include "scenario_runner.h"
#include "test_scenarios.h"
#include "calibration_runner.h"
#include "test_calibration.h"
#include "test_loop_status_json.h"
#include "test_device_config.h"
#include "test_device_logs.h"
#include "test_asiair_parser.h"
#include "test_pa_tracker.h"
#include "test_block_c_json.h"
#include "test_ota_rules.h"
#include "test_ota_gate.h"
#include "test_backlash_planner.h"
#include "test_nina_bridge.h"
#include "test_loop_backlash.h"

int main() { return ttRunAll(); }
