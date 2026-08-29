#include "autons.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

#include <cmath>
#include <cstdio>

/* HOW TO WRITE A ROUTINE

   Plain LemLib. Every motion below is a real call out of
   include/lemlib/chassis/chassis.hpp, so lemlib.readthedocs.io and anything
   another team posts applies here without translation.

   The last argument of every motion is `async`. Pass false and the call blocks
   until the motion finishes, so the routine reads straight down the page --
   one line, one thing the robot does, in order:

     chassis.moveToPoint(-24, 0, 1500, {}, false);   // drives, then returns
     chassis.turnToHeading(90, 900, {}, false);      // turns, then returns
     intake.move(127);                               // instant, no waiting

   Leave that false off and the call returns immediately, so the next line runs
   while the robot is still moving. Useful when you want the lift going up
   during a drive; confusing everywhere else.

   The middle {} is the params struct. Fill in only what you want to change:

     chassis.moveToPoint(x, y, t, {.forwards = false}, false);    // reverse
     chassis.moveToPoint(x, y, t, {.maxSpeed = 70}, false);       // slower
     chassis.moveToPose(x, y, heading, t, {.lead = 0.4f}, false); // curve in

   The number before the params is the timeout in milliseconds -- a safety net,
   not a schedule. Give a motion roughly twice as long as it should need.

   COORDINATES are inches from field centre. +Y is away from the driver
   station, +X is to its right, heading 0 faces +Y and increases clockwise.
   Nothing is mirrored for you, which is why red and blue are separate
   functions.

   Every routine sets its own start pose, because odometry has no idea where
   you put the robot on the tile. */

namespace auton {
namespace {
constexpr double PI = 3.14159265358979;

// Backs into whatever is behind the robot, lifts to `height` (0 to 1 of full
// travel), spits, and drops back to travel height. Ordinary C++ taking
// ordinary arguments -- the way to share behaviour between routines.
void score_backwards(double into_inches, double height) {
  lemlib::Pose p = chassis.getPose();
  double t = p.theta * PI / 180.0;

  chassis.moveToPoint(p.x - std::sin(t) * into_inches, p.y - std::cos(t) * into_inches, 1500,
                      {.forwards = false}, false);

  lift.move_absolute(height * LIFT_TICKS, 100);
  pros::delay(500);

  intake.move(-127);
  claw_spin.move(-127);
  pros::delay(600);
  intake.move(0);
  claw_spin.move(0);

  lift.move_absolute(LIFT_TRAVEL * LIFT_TICKS, 100);
  pros::delay(400);
}
} // namespace

// Holds still for the whole autonomous period. First in the selector list so a
// robot nobody configured sits there instead of running whatever was picked
// last.
void do_nothing() { chassis.arcade(0, 0); }

/* !! PLACEHOLDER STRATEGY. The two routines below drive to the toggle on their
   own side and come back. The path is closed loop and will land where the
   numbers say, but the numbers themselves are a guess -- nobody has decided
   what these should do at a match yet. */

// West toggle sits at (-68, 0). Red starts at (-52, 0) facing +X, so this
// turns around, closes on it, and backs off to leave the driver room.
// my_route -- designed in Catpath, 5 steps
// red alliance. Absolute field coordinates, inches from centre,
// LemLib headings. Nothing here mirrors: export again from the other side.
// my_route -- designed in Catpath, 5 steps
// red alliance. Absolute field coordinates, inches from centre,
// LemLib headings. Nothing here mirrors: export again from the other side.
// my_route -- designed in Catpath, 12 steps
// red alliance. Absolute field coordinates, inches from centre,
// LemLib headings. Nothing here mirrors: export again from the other side.
// my_route -- designed in Catpath, 12 steps
// red alliance. Absolute field coordinates, inches from centre,
// LemLib headings. Nothing here mirrors: export again from the other side.
void my_route() {
  chassis.setPose(-72, 0, -90);

 
 
  chassis.moveToPoint(-65, 0, 852, {.forwards = false}, false);
    claw_pivot.move_absolute(-850, 127);

  // 8 in
  
  chassis.moveToPoint(-76, 0, 852, {}, false);
  pros::delay(600);
  // 33.5 in
  chassis.moveToPoint(-51.25, -20.50, 1776, {.forwards = false}, false);
  intake.move(127);
  claw_spin.move(127);
  pros::delay(900);


  chassis.moveToPoint(-20.25, -18.25, 1882, {}, false);
  chassis.turnToHeading(39, 1000, {}, false);

  lift.move_absolute(600, 100);
    chassis.moveToPoint(-21.25, -19.25, 1882, {}, false);




}
// East toggle sits at (68, 0). Blue starts at (-52, 0) facing +Y, so this is
// not a mirror of red_toggle -- it is its own route, which is the whole reason
// they are separate functions.
void blue_toggle() {
  std::printf("blue toggle: start\n");
  chassis.setPose(-52, 0, 0);

  chassis.turnToHeading(90, 1500, {}, false);
  chassis.moveToPoint(0, 0, 3000, {}, false);
  chassis.moveToPoint(62, 0, 3000, {}, false);

  intake.move(127);
  claw_spin.move(127);
  pros::delay(600);
  intake.move(0);
  claw_spin.move(0);

  chassis.moveToPoint(48, 0, 1500, {.forwards = false}, false);
  std::printf("blue toggle: done\n");
}

// A compiling example of the shape a routine takes. Delete it once there are
// real ones.
void example() {
  std::printf("example: start\n");
  chassis.setPose(-52, 0, 90);

  lift.move_absolute(LIFT_TRAVEL * LIFT_TICKS, 100);
  intake.move(127);

  chassis.moveToPoint(-32, 0, 2000, {}, false);
  pros::delay(300);
  intake.move(0);

  chassis.turnToHeading(90, 1000, {}, false);
  score_backwards(12, 0.30);

  chassis.moveToPoint(-30, 0, 2000, {}, false);
  std::printf("example: done\n");
}

} // namespace auton
