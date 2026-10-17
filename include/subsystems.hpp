#pragma once

#include "EZ-Template/api.hpp" // IWYU pragma: keep
#include "main.h"              // IWYU pragma: keep

// Every motor, sensor and number that describes this robot. Ports live in
// subsystems.cpp and nowhere else.

// ---------------------------------------------------------------- drivetrain
// EZ-Template's Drive owns the motors, so the groups the rest of the code
// reaches for are chassis.left_motors / chassis.right_motors.
extern Drive chassis;

// ------------------------------------------------------------------- sensors
extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;
extern pros::Rotation claw; // measures the claw pivot itself

extern ez::tracking_wheel horizontal_tracker;

// --------------------------------------------------- numbers that describe it
const int IMU_PORT = 20;
const double WHEEL_DIA = 3.4211;  // 24 in drive read 22.8
const double DRIVE_TICKS = 360;   // blue cartridge, after gearing
const double TRACK_WIDTH = 11.5;  // between the drive wheel contact patches

// ---------------------------------------------------------------- mechanisms
extern pros::MotorGroup lift;
extern pros::Motor claw_pivot;
extern pros::Motor claw_spin;
extern pros::Motor intake;

// ------------------------------------------------------- numbers worth changing
const double LIFT_TICKS = 900;    // motor degrees, bottom to top
const double LIFT_TRAVEL = 0.15;  // ride height while driving, 0 to 1
const double LIFT_TOP = 2700;     // do not drive the lift above this
const double LIFT_BOTTOM = -50;   // or below this
const double CLAW_DRIFT_DEG = 15; // slip allowed before the claw is put back

// Above this the claw tucks itself to position 1.
const double LIFT_CLAW_DEG = 200;


// Claw pivot speed in RPM. 200 is flat out for a green cartridge -- the second
// argument of move_absolute is a velocity, and PROS clamps it to whatever the
// gearset can do. Lower this to make the claw move more gently.
const int CLAW_SPEED = 200;

// Claw pivot targets in motor degrees, one per position. Measured with the
// robot powered on, lift down and claw pointing down -- that is where
// claw_pivot.tare_position() puts zero.
extern const double CLAW_POS[3];

// False until chassis.initialize() has finished. initialize() calibrates the
// IMU and starts the odometry task, so ANY odom_xyt_set before it completes is
// thrown away -- which makes a route drive from a position it was never at.
extern volatile bool chassis_ready;

// re-home: drive this far past pos 0, settle, then call that spot pos 0
const double CLAW_HOME_PAST = 200;
const uint32_t CLAW_HOME_GIVEUP_MS = 1800;
const uint32_t CLAW_HOME_SETTLE_MS = 20;

// Drive the claw down past pos 0, then call that the new zero. Returns
// straight away, claw_update() finishes it off.
void claw_home();

// Move the claw to position 0, 1 or 2. Returns immediately.
void spinclaw(int position);

// Call every loop. Stops the claw straining once it arrives, and puts it back
// if the rotation sensor says something knocked it off.
void claw_update();

// The position spinclaw() was last asked for.
int claw_at();
