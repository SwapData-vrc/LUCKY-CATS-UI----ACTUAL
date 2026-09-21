#include "subsystems.hpp" // IWYU pragma: keep

pros::MotorGroup left_motors({-7, -5}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({4, 6}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(&left_motors, &right_motors,
                              11.5,                         // track width, inches
                              lemlib::Omniwheel::NEW_325, // wheel
                              360, // wheel RPM after gearing
                              2    // horizontal drift
);

pros::Imu imu(20);

pros::Rotation horizontal_encoder(-19);
pros::Rotation vertical_rotation(-13);

// Measures the claw pivot itself, so claw_update() can tell when something
// has knocked the claw off the position it was holding.
pros::Rotation claw(1);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_275,
                                                -5.75);

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation,
                                              lemlib::Omniwheel::NEW_2, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr,
                            &horizontal_tracking_wheel, nullptr, &imu);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(
    22,
    0,
    10,
    0,
    2,
    100,
    12,    // large error range
    1500,  // large error timeout
    100
);
// angular PID controller
lemlib::ControllerSettings angular_controller(
    15,   // kP
    0,    // kI
    125,    // kD
    0,
    1,
    100,
    3,
    500,
    127
);
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors);

pros::MotorGroup lift({-18, 2}, pros::MotorGearset::green);
pros::Motor claw_pivot(-3, pros::MotorGearset::green);
pros::Motor claw_spin(1, pros::MotorGearset::green);
pros::Motor intake(9, pros::MotorGearset::blue);

volatile bool chassis_ready = false;

// --------------------------------------------------------------------- claw
const double CLAW_POS[3] = {180, -850, -1405};

namespace {
int g_position = 0;      // which of the three we were last told to go to
double g_target = 0;     // CLAW_POS[g_position], in motor degrees
uint32_t g_started = 0;  // when that move was commanded
bool g_holding = false;  // has it stopped moving and latched a reference yet
double g_hold_angle = 0; // rotation sensor reading when it stopped, degrees

// The claw angle from the rotation sensor on port 1, which counts in
// hundredths of a degree.
double claw_angle() { return claw.get_position() / 100.0; }
} // namespace

int claw_at() { return g_position; }

void spinclaw(int position) {
  if (position < 0 || position > 2) return;

  g_position = position;
  g_target = CLAW_POS[position];
  g_started = pros::millis();
  g_holding = false;

  claw_pivot.move_absolute(g_target, CLAW_SPEED);
  claw_pivot.move_absolute(g_target, 1000);
}




void claw_update() {
  // Nothing to do until it either arrives or gives up trying.
  if (!g_holding) {
    const bool arrived = std::fabs(g_target - claw_pivot.get_position()) < 5;
    const bool stuck = pros::millis() - g_started > 1500;
    if (!arrived && !stuck) return;

    claw_pivot.move(0);
    g_holding = true;

    // The reference is where it actually stopped, not a number written down
    // here: the sensor's zero depends on how the claw was bolted on.
    g_hold_angle = claw_angle();
    return;
  }

  // Position 0 is the claw resting down. Nothing holds it there and nothing
  // needs to -- drift is just the mechanism sitting where gravity puts it.
  if (g_position == 0) return;

  // No sensor, nothing to correct against. Without this an unplugged sensor
  // reads PROS_ERR and the claw re-commands itself every single loop.
  if (claw.get_position() == PROS_ERR) return;

  // Knocked, or sagging under what it is holding. Re-issuing the same command
  // clears g_holding, so it settles and latches a fresh reference. If it
  // physically cannot get back, the 1500 ms above stops it fighting.
  if (std::fabs(claw_angle() - g_hold_angle) > CLAW_DRIFT_DEG) spinclaw(g_position);
}
