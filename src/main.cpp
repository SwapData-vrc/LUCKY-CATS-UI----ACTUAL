#include "main.h"
#include "screen.hpp"
#include "subsystems.hpp"

#include <cstdio>
#include <cstdlib>

namespace {
// Sticks do not come back to exactly zero. Without this the robot creeps
// across the tile with nobody touching the controller.
constexpr int DEADBAND = 5;

// Was the lift up last loop? Used to fire the claw tuck once, on the way up.
bool lift_was_up = false;
} // namespace

void initialize() {
  // Screen first. PROS does not start opcontrol() until initialize() returns,
  // so anything slow in front of this is a black brain and a dead controller.
  screen::init();

  // The claw positions are measured from here, so the robot has to be powered
  // on with the lift down and the claw pointing down.
  lift.tare_position();
  claw_pivot.tare_position();

  // Brake, not coast: on coast the drive rolls on past where the loop thinks
  // it stopped, and every later step of a route inherits that error.
  left_motors.set_brake_mode(pros::MotorBrake::brake);
  right_motors.set_brake_mode(pros::MotorBrake::brake);

  // Hold, not coast: the lift sinks under its own weight on coast.
  lift.set_brake_mode(pros::MotorBrake::hold);
  claw_pivot.set_brake_mode(pros::MotorBrake::hold);
  intake.set_brake_mode(pros::MotorBrake::coast);
  claw_spin.set_brake_mode(pros::MotorBrake::coast);

  // Its own task so a slow or unplugged IMU cannot keep the driver waiting.
  // arcade() is open loop and does not use odometry, so driving while this
  // finishes is safe. autonomous() waits for it, because routes are odometry.
  pros::Task calibrate_task([] {
    chassis.calibrate();
    chassis_ready = true;
    std::printf("chassis calibrated\n");
  });
}

void disabled() {
  chassis.cancelAllMotions();
  intake.move(0);
  claw_spin.move(0);
  lift.brake();
}

void competition_initialize() {}

void autonomous() {
  while (!chassis_ready) pros::delay(10);
  screen::run_selected();
}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  // The field kills the autonomous task, but LemLib's chassis task does not
  // know that and would keep driving its last motion into the driver period.
  chassis.cancelAllMotions();

  chassis.setBrakeMode(MOTOR_BRAKE_COAST);
  while (true) {
    // ---- drive ----
    // Hand the drive over while a route started from this screen is running.
    // Without this, arcade() re-commands the motors every 25 ms and fights
    // LemLib for the whole route -- the robot twitches and goes nowhere, which
    // reads as "the run button does not work".
    if (!screen::running()) {
      int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

      if (std::abs(leftY) < DEADBAND) leftY = 0;
      if (std::abs(rightX) < DEADBAND) rightX = 0;

      chassis.arcade(leftY, rightX);
    }

    // LEFT + A starts the selected routine, for testing on a practice field.
    // Both buttons, because one of them is a stray press and two is a
    // decision. request_run() refuses under competition control, so this
    // cannot fire at an event.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      if (screen::running()) screen::request_stop();
      else screen::request_run();
    }

    // ---- claw ----
    // B steps through the three positions.
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
      spinclaw((claw_at() + 1) % 3);

    // Once the lift is up the claw belongs at position 1. Fires on the way
    // past, not every loop, so B still works afterwards.
    const bool lift_up = lift.get_position() > LIFT_CLAW_DEG;
    if (lift_up && !lift_was_up) spinclaw(1);
    lift_was_up = lift_up;

    claw_update();

    // ---- intake ----
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake.move(100);


      claw_spin.move(-50);

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {

       intake.move(-100);

      claw_spin.move(50);
    } else {
      intake.move(0);
      claw_spin.move(0);
    }

    // ---- lift ----
    const double height = lift.get_position();
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && height < LIFT_TOP) lift.move(110);
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && height > LIFT_BOTTOM) lift.move(-90);
    else lift.brake();

    // Required. Without it this loop never yields and the screen and the
    // competition task are starved.
    pros::delay(25);
  }
}
