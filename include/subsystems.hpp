#pragma once

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"         // IWYU pragma: keep

// Every motor, sensor and number that describes this robot. Ports live in
// subsystems.cpp and nowhere else.

// ---------------------------------------------------------------- drivetrain
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern lemlib::Drivetrain drivetrain;

// ------------------------------------------------------------------- sensors
extern pros::Imu imu;
extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;
extern pros::Rotation claw; // measures the claw pivot itself

extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::OdomSensors sensors;

// --------------------------------------------------------------------- chassis
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis;

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
const double LIFT_CLAW_DEG = 200; // above this the claw tucks itself to 1
const double CLAW_DRIFT_DEG = 15; // slip allowed before the claw is put back

// Claw pivot speed in RPM. 200 is flat out for a green cartridge -- the second
// argument of move_absolute is a velocity, and PROS clamps it to whatever the
// gearset can do. Lower this to make the claw move more gently.
const int CLAW_SPEED = 200;

// Claw pivot targets in motor degrees, one per position. Measured with the
// robot powered on, lift down and claw pointing down -- that is where
// claw_pivot.tare_position() puts zero.
extern const double CLAW_POS[3];

// False until chassis.calibrate() has finished. calibrate() zeroes the pose
// and starts the odometry task, so ANY setPose before it completes is thrown
// away -- which makes a route drive from a position it was never at.
extern volatile bool chassis_ready;

// Move the claw to position 0, 1 or 2. Returns immediately.
void spinclaw(int position);

// Call every loop. Stops the claw straining once it arrives, and puts it back
// if the rotation sensor says something knocked it off.
void claw_update();

// The position spinclaw() was last asked for.
int claw_at();
