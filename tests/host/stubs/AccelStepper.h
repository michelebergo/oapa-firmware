// Host model of AccelStepper: positions and targets are exact, run() advances
// one step, stop() lands the target where the axis is.
#pragma once
#include <cstdint>

class AccelStepper {
 public:
  enum MotorInterfaceType { DRIVER = 1 };
  AccelStepper(uint8_t = DRIVER, uint8_t = 2, uint8_t = 3) {}
  void move(long relative) { target_ = pos_ + relative; }
  void moveTo(long absolute) { target_ = absolute; }
  long currentPosition() { return pos_; }
  long targetPosition() { return target_; }
  long distanceToGo() { return target_ - pos_; }
  void setCurrentPosition(long p) { pos_ = target_ = p; speed_ = 0; }
  void setMaxSpeed(float s) { maxSpeed_ = s; }
  float maxSpeed() { return maxSpeed_; }
  void setAcceleration(float a) { accel_ = a; }
  void setSpeed(float s) { speed_ = s; }
  float speed() { return speed_; }
  bool isRunning() { return target_ != pos_; }
  void stop() { target_ = pos_; }
  bool run() {
    if (pos_ == target_) return false;
    pos_ += target_ > pos_ ? 1 : -1;
    return true;
  }
  bool runSpeed() { return false; }

 private:
  long pos_ = 0, target_ = 0;
  float maxSpeed_ = 1, accel_ = 1, speed_ = 0;
};
