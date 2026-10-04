#include "main.h"
#include "record.hpp"
#include "screen.hpp"
#include "subsystems.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
// Sticks do not come back to exactly zero. Without this the robot creeps
// across the tile with nobody touching the controller.
constexpr int DEADBAND = 5;

// set from the real height when we take over, else a route that ends
// with the lift up looks like the driver just raised it
bool lift_was_up = false;

// ------------------------------------------------------------- heading hold
// arcade() is open loop: both sides of the drive get the same number. Anything
// that makes one side stronger than the other -- a tired motor, a tight
// bearing, weight sitting off centre -- shows up as the robot curving away on
// a straight stick. This closes the loop on the IMU while the driver is going
// forward and not asking for a turn.
constexpr bool HEADING_HOLD = true; // set false to go back to raw arcade

// Turn units per degree of error. Higher snaps back harder and starts to
// wobble; lower lets the robot wander further before it argues.
constexpr double HEADING_KP = 2.0;

// The correction never exceeds this, so a confused IMU cannot take the robot
// away from the driver.
constexpr int HEADING_MAX = 40;

// Yaw rate below which the robot counts as no longer turning, degrees/sec.
// The heading is only latched once it is under this -- a robot still carrying
// rotation from the last flick of the turn stick would otherwise lock in a
// heading nobody meant to hold.
constexpr double SETTLED_DPS = 20.0;

bool holding = false;      // is a heading currently latched
double hold_heading = 0;   // the heading being held, degrees

// Shortest signed way round from `from` to `to`, in [-180, 180] degrees, so
// the correction never takes the long way round through 0/360.
double heading_error(double to, double from) {
  return std::fmod(to - from + 540.0, 360.0) - 180.0;
}
} // namespace

void initialize() {
  // Screen first. PROS does not start opcontrol() until initialize() returns,
  // so anything slow in front of this is a black brain and a dead controller.
  screen::init();

  // a dead tracker makes every motion run to timeout, so say so at boot
  std::printf("vertical  tracker port 12: %s\n",
              vertical_encoder.get_position() == PROS_ERR ? "NOT REPORTING" : "ok");
  std::printf("horizontal tracker port 19: %s\n",
              horizontal_encoder.get_position() == PROS_ERR ? "NOT REPORTING" : "ok");
  std::printf("lift       ports 18,2:  %s\n",
              lift.get_position() == PROS_ERR_F ? "NOT REPORTING" : "ok");
  std::printf("claw       sensor port 13: %s\n",
              claw.get_position() == PROS_ERR ? "NOT REPORTING" : "ok");

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
    // encoders survive a restart
    horizontal_encoder.reset_position();
    left_motors.tare_position();
    right_motors.tare_position();

    chassis.calibrate();

    chassis.setPose(0, 0, 0);

    chassis_ready = true;
    std::printf("chassis calibrated, pose 0 0 0\n");
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

  // Coast is what the drivers want under the sticks. Routes want brake, and
  // swap back and forth below -- see route_owns_drive.
  chassis.setBrakeMode(MOTOR_BRAKE_COAST);

  // True while a screen-started route has the drive, so the brake mode is
  // swapped exactly once on each edge instead of every 25 ms.
  bool route_owns_drive = false;

  lift_was_up = lift.get_position() > LIFT_CLAW_DEG;

  while (true) {
    // A route started from this screen owns every motor for as long as it
    // runs, so the whole manual block below is skipped. Without this the loop
    // re-commands the motors every 25 ms and undoes whatever the route just
    // asked for: arcade() fights LemLib for the drive, and intake.move(0),
    // claw_spin.move(0), lift.brake() and claw_update() wipe out the intake,
    // claw and lift calls in the route within one loop -- which reads as
    // "the drive works but nothing else in my route does".
    //
    // Nothing needs undoing when the route ends. The next pass through has
    // screen::running() false again, and with no buttons held the else
    // branches below zero the intake and brake the lift on their own.
    if (screen::running()) {
      // Brake, not coast. A route that ends on coast rolls on past where the
      // loop thinks it stopped, and every later step inherits that error --
      // the same reason initialize() sets brake before autonomous().
      if (!route_owns_drive) {
        chassis.setBrakeMode(MOTOR_BRAKE_BRAKE);
        route_owns_drive = true;
      }

      // LEFT + A again stops it. Same chord as starting one, below.
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
          master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
        screen::request_stop();

      pros::delay(25);
      continue;
    }

    // The route just ended, so give the drivers their coast back.
    if (route_owns_drive) {
      chassis.setBrakeMode(MOTOR_BRAKE_COAST);
      route_owns_drive = false;
      holding = false; // whatever heading was latched before the route is stale
      lift_was_up = lift.get_position() > LIFT_CLAW_DEG;
    }

    // ---- drive ----
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (std::abs(leftY) < DEADBAND) leftY = 0;
    if (std::abs(rightX) < DEADBAND) rightX = 0;

    int turn = rightX;

    // Hold the heading while the driver is going somewhere and not steering.
    // Any turn input at all hands steering straight back, on the same loop.
    // cheap checks first, saves a port read when hold cant kick in anyway
    const bool want_hold = HEADING_HOLD && chassis_ready && rightX == 0 && leftY != 0;
    const double now = want_hold ? imu.get_heading() : 0.0;
    if (want_hold && std::isfinite(now)) {
      if (!holding) {
        const double spin = imu.get_gyro_rate().z;
        if (std::isfinite(spin) && std::fabs(spin) < SETTLED_DPS) {
          hold_heading = now;
          holding = true;
        }
      }

      if (holding) {
        double correction = heading_error(hold_heading, now) * HEADING_KP;
        if (correction > HEADING_MAX) correction = HEADING_MAX;
        if (correction < -HEADING_MAX) correction = -HEADING_MAX;
        turn = static_cast<int>(correction);
      }
    } else {
      holding = false;
    }

    chassis.arcade(leftY, turn);

    // LEFT + A starts the selected routine, for testing on a practice field.
    // Both buttons, because one of them is a stray press and two is a
    // decision. request_run() refuses under competition control, so this
    // cannot fire at an event.
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
        master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
      screen::request_run();

    // RIGHT+X = record on/off, RIGHT+Y = print it. two buttons so you cant
    // knock it on mid match. same idea as LEFT+A for routes.
    const bool rec_mod = master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);

    if (rec_mod && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      record::toggle();
      master.rumble(record::active() ? "." : "..");
    }

    // RIGHT+Y builds the code and shows it on the brain, a page at a time.
    // keep pressing to page through, it hides itself at the end.
    if (rec_mod && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      if (record::line_count() == 0) record::print_last();
      screen::show_code_page();
      master.rumble("-");
    }

    // UP drives the claw all the way down and calls that zero again
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
      claw_home();

    // ---- claw ----
    // B steps through the three positions.
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
      spinclaw((claw_at() + 1) % 3);

    const double lift_h = lift.get_position();

    // ---- claw follows the lift ----
    const bool lift_up = lift_h > LIFT_CLAW_DEG;
    if (lift_up && !lift_was_up) spinclaw(1);
    lift_was_up = lift_up;

    claw_update();

    // ---- intake ----
    int intake_cmd = 0, spin_cmd = 0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intake_cmd = 100;
      spin_cmd = -50;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake_cmd = -100;
      spin_cmd = 50;
    }
    intake.move(intake_cmd);
    claw_spin.move(spin_cmd);

    // ---- lift ----
    const bool l1 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    const bool l2 = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    int lift_cmd = 0;
    if (l1 && lift_h < LIFT_TOP) lift_cmd = 127;
    else if (l2 && lift_h > LIFT_BOTTOM) lift_cmd = -127;

    if (lift_cmd != 0) lift.move(lift_cmd);
    else lift.brake();

    record::update(lift_cmd, intake_cmd, spin_cmd);

    // Required. Without it this loop never yields and the screen and the
    // competition task are starved.
    pros::delay(25);
  }
}
