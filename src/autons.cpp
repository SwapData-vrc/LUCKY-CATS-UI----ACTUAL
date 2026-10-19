#include "autons.hpp" // IWYU pragma: keep
#include "main.h"      // IWYU pragma: keep
#include "subsystems.hpp"

// Autonomous routines live here.
//
// Speeds are out of 127. EZ-Template takes its exit conditions and PID
// constants from its own defaults until you set them, so a routine only needs
// the motions themselves.
//
//   chassis.odom_xyt_set(0_in, 0_in, 0_deg);
//   chassis.pid_drive_set(24_in, DRIVE_SPEED);
//   chassis.pid_wait();
//   chassis.pid_turn_set(90_deg, TURN_SPEED);
//   chassis.pid_wait();
//   chassis.pid_odom_set({{24_in, 24_in, 90_deg}, fwd, DRIVE_SPEED});
//   chassis.pid_wait();
//
// Declare each routine in autons.hpp and list it in ROUTINES in screen.cpp so
// it shows up on the brain.

const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

namespace auton {} // namespace auton
