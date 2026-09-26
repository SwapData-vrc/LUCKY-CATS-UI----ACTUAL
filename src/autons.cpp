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

// skills -- multi-cycle scoring routine.
void skills() {
  chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
  spinclaw(1);
  chassis.moveToPoint(0, -2.3, 552, {.forwards = false, .maxSpeed = 70}, false);
  chassis.turnToHeading(-90, 700, {.maxSpeed = 80}, false);
  chassis.moveToPoint(10, -2.3, 852, {.forwards = false, .maxSpeed = 70},
                      false);
  claw_spin.move(127);
  pros::delay(1000);
  spinclaw(2);
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(5, 100, {.maxSpeed = 80}, false);
  chassis.setPose(0, 0, 0);

  intake.move(-127);
  chassis.moveToPoint(0, 23, 968, {.maxSpeed = 70}, false);
  spinclaw(0);
  chassis.turnToHeading(180, 2000, {.maxSpeed = 80}, false);
  chassis.moveToPoint(0, 60, 1572, {.forwards = false, .maxSpeed = 80}, false);
  chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
  chassis.moveToPoint(0, 0, 572, {.maxSpeed = 80}, false);
  chassis.turnToHeading(-130, 800, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-30, -3, 3572, {.maxSpeed = 80}, false);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, -0.5, 100, {.forwards = false, .maxSpeed = 80}, false);
  pros::delay(500);
  chassis.moveToPoint(0, -2.3, 552, {.forwards = false, .maxSpeed = 70}, false);
  chassis.turnToHeading(-90, 700, {.maxSpeed = 80}, false);
  spinclaw(1);
  lift.move_absolute(900, 127);
  pros::delay(100);
  chassis.moveToPoint(20, -2.3, 1852, {.forwards = false, .maxSpeed = 70},
                      false);
  lift.move_absolute(200, 127);
  pros::delay(100);
  claw_spin.move(127);
  pros::delay(10);
  lift.move_absolute(900, 127);
  pros::delay(100);
  /*

  chassis.setPose(0,0,0);
  chassis.moveToPoint(0, 3, 1000, { .maxSpeed = 80}, false);
  chassis.turnToHeading(20, 700, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-4, -6, 1000, {.forwards = false, .maxSpeed = 80}, false);
  chassis.turnToHeading(90, 700, {.maxSpeed = 80}, false);
  chassis.setPose(0, 0, 0);

  pros::delay(1000000);


  */

  chassis.setPose(0, 0, 0);

  intake.move(-127);
  chassis.moveToPoint(0, 23, 968, {.maxSpeed = 70}, false);
  spinclaw(0);
  lift.move_absolute(-20, 127);
  chassis.turnToHeading(180, 2000, {.maxSpeed = 80}, false);
  chassis.moveToPoint(0, 60, 1572, {.forwards = false, .maxSpeed = 80}, false);
  chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
  chassis.moveToPoint(0, 0, 572, {.maxSpeed = 80}, false);
  chassis.turnToHeading(-130, 800, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-30, -3, 3572, {.maxSpeed = 80}, false);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, -0.5, 100, {.forwards = false, .maxSpeed = 80}, false);
  pros::delay(500);
  chassis.moveToPoint(0, -2.3, 552, {.forwards = false, .maxSpeed = 70}, false);
  chassis.turnToHeading(-90, 700, {.maxSpeed = 80}, false);
  spinclaw(1);
  lift.move_absolute(1500, 127);
  pros::delay(100);
  chassis.moveToPoint(20, -2.3, 1852, {.forwards = false, .maxSpeed = 70},
                      false);
  lift.move_absolute(800, 127);
  pros::delay(100);
  claw_spin.move(127);
  pros::delay(10);
  lift.move_absolute(1500, 127);
  pros::delay(100);

  chassis.setPose(0, 0, 0);

  intake.move(-127);
  chassis.moveToPoint(0, 23, 968, {.maxSpeed = 70}, false);
  spinclaw(0);
  lift.move_absolute(-20, 127);
  chassis.turnToHeading(180, 2000, {.maxSpeed = 80}, false);
  chassis.moveToPoint(0, 60, 1572, {.forwards = false, .maxSpeed = 80}, false);
  chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
  chassis.moveToPoint(0, 0, 572, {.maxSpeed = 80}, false);
  chassis.turnToHeading(-130, 800, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-30, -3, 3572, {.maxSpeed = 80}, false);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, -0.5, 100, {.forwards = false, .maxSpeed = 80}, false);
  pros::delay(500);
  chassis.moveToPoint(0, -2.3, 552, {.forwards = false, .maxSpeed = 70}, false);
  chassis.turnToHeading(-90, 700, {.maxSpeed = 80}, false);
  spinclaw(1);
  lift.move_absolute(1700, 127);
  pros::delay(100);
  chassis.moveToPoint(20, -2.3, 1852, {.forwards = false, .maxSpeed = 70},
                      false);
  lift.move_absolute(1000, 127);
  pros::delay(100);
  claw_spin.move(127);
  pros::delay(10);
  lift.move_absolute(1700, 127);
  pros::delay(100);

  chassis.setPose(0, 0, 0);

  intake.move(-127);
  chassis.moveToPoint(0, 23, 968, {.maxSpeed = 70}, false);
  spinclaw(0);
  lift.move_absolute(-20, 127);
  chassis.turnToHeading(180, 2000, {.maxSpeed = 80}, false);
  chassis.moveToPoint(0, 60, 1572, {.forwards = false, .maxSpeed = 80}, false);
  chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
  chassis.moveToPoint(0, 0, 572, {.maxSpeed = 80}, false);
  chassis.turnToHeading(-130, 800, {.maxSpeed = 80}, false);
  chassis.moveToPoint(-30, -3, 3572, {.maxSpeed = 80}, false);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, -0.5, 100, {.forwards = false, .maxSpeed = 80}, false);
  pros::delay(500);
  chassis.moveToPoint(0, -2.3, 552, {.forwards = false, .maxSpeed = 70}, false);
  chassis.turnToHeading(-90, 700, {.maxSpeed = 80}, false);
  spinclaw(1);
  lift.move_absolute(2100, 127);
  pros::delay(100);
  chassis.moveToPoint(20, -2.3, 1852, {.forwards = false, .maxSpeed = 70},
                      false);
  lift.move_absolute(1500, 127);
  pros::delay(100);
  claw_spin.move(127);
  pros::delay(10);
  lift.move_absolute(2100, 127);
  pros::delay(100);

  /*        chassis.setPose(-72, 0, -90);
   intake.move(127);
   claw_spin.move(-127);

   chassis.moveToPoint(-65, 0, 852, {.forwards = false}, false);

   // 8 in

   chassis.moveToPoint(-76, 0, 852, {}, false);
   pros::delay(600);

   chassis.moveToPoint(-65, 0, 852, {.forwards = false}, false);

   // 8 in
   spinclaw(1);
   chassis.moveToPoint(-76, 0, 852, {}, false);
   pros::delay(600);

   //  33.5 in

   chassis.moveToPoint(-51.25, -21.70, 1876, {.forwards = false}, false);
   intake.move(127);
   claw_spin.move(127);
   pros::delay(900);
 spinclaw(2);
 pros::delay(10);
   lift.move_absolute(0, 100);
   pros::delay(100);
   claw_spin.move(-127);

   chassis.setPose(0, 0, 0);
   chassis.moveToPoint(0, 12, 1000, {.maxSpeed = 80}, false);
   claw_spin.move(-127);
   chassis.turnToHeading(-42.50, 850, {.maxSpeed = 80}, false);
   pros::delay(100);

   // Safe here only because the turn above is blocking. setPose while a motion
   // is still running moves the goalposts under the controller that is chasing
   // them.
   chassis.setPose(0, 0, 0);
   spinclaw(2);

   chassis.moveToPoint(0, -34, 3104, {.forwards = false, .maxSpeed = 23.9},
 false); chassis.waitUntil(-26); lift.move_absolute(0, 70);
   claw_spin.move(-127);

   chassis.turnToPoint(12, -1.1, 200, {.maxSpeed = 40}, false);
  pros::delay(100);
   spinclaw(0);
   pros::delay(800);
   spinclaw(1);
   lift.move_absolute(1200, 127);
   pros::delay(100);
   chassis.setPose(0, 0, 0);
   chassis.turnToHeading(101, 850, {.maxSpeed = 80}, false);

   chassis.setPose(0, 0, 0);
   chassis.moveToPoint(0, -24, 1000, {.forwards = false, .maxSpeed = 80},
 false); lift.move_absolute(100, 127); pros::delay(40); claw_spin.move(127);
   pros::delay(800);
   lift.move_absolute(1300, 127);
   // end 2nd score
   chassis.moveToPoint(0, 2, 1000, {.maxSpeed = 80}, false);
   chassis.setPose(0, 0, 0);

   chassis.turnToHeading(-40.8, 850, {.maxSpeed = 80}, false);
   chassis.setPose(0, 0, 0);
   lift.move_absolute(-90, 70);
   pros::delay(100);

   spinclaw(2);
   lift.move_absolute(0, 70);
   chassis.moveToPoint(0, -32.5, 1571, {.forwards = false, .maxSpeed = 40},
                       false);

   chassis.waitUntil(-27.5);
   lift.move_absolute(0, 70);
   claw_spin.move(-127);

   chassis.turnToHeading(13, 700, {.maxSpeed = 127}, false);
   spinclaw(0);
   pros::delay(600);
   spinclaw(1);
   lift.move_absolute(8500, 127);
   pros::delay(90);
   chassis.setPose(0, 0, 0);
   chassis.turnToHeading(129, 850, {.maxSpeed = 80}, false);

   chassis.setPose(0, 0, 0);
   chassis.moveToPoint(0, -27, 1000, {.forwards = false, .maxSpeed = 50},
 false); lift.move_absolute(1300, 127);

   claw_spin.move(-127);
   pros::delay(1000);
   lift.move_absolute(400, 127);
   pros::delay(100);
   claw_spin.move(127);
   lift.move_absolute(1500, 127);
   pros::delay(100);
   */
}

// test -- angular PID check. 180, then 270, then back to 0. Each leg settles
// before the next starts, so overshoot on one turn does not feed the next.
void test() {
  chassis.setPose(0, 0, 0);

  chassis.turnToHeading(180, 2000, {}, false);
  pros::delay(200);
  chassis.turnToHeading(270, 2000, {}, false);
  pros::delay(200);
  chassis.turnToHeading(0, 2000, {}, false);
}

// 30pts -- scoring routine. Empty on purpose: the motions have not been
// decided yet, and a half-guessed auton is worse than one that does nothing.
void thirty_pts() {
  chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
  chassis.moveToPoint(0, -5, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 4, 450, {.minSpeed = 100}, false);
  pros::delay(10);
  chassis.moveToPoint(0, -5, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 4, 450, {.minSpeed = 100}, false);
  pros::delay(10);
  spinclaw(1);
  chassis.moveToPoint(-24, -9.8, 4000, {.forwards = false, .minSpeed = 100},
                      false);
  claw_spin.move(127);
  pros::delay(700);
  chassis.setPose(0, 0, 0);
  spinclaw(2);
  pros::delay(50);
  claw_spin.move(-127);
  chassis.moveToPoint(0, 10.3, 650, {.maxSpeed = 100}, false);
  chassis.turnToHeading(-146, 300, {.maxSpeed = 127}, false);
  chassis.setPose(-5, 0, 0);
  // exits 1 in short so the turn does not spike at the point
  chassis.moveToPoint(0, -12, 1250,
                      {.forwards = false, .maxSpeed = 30, .minSpeed = 10,
                       .earlyExitRange = 1},
                      true);
  chassis.waitUntil(7);
  spinclaw(0);
  pros::delay(700);
  spinclaw(1);

  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
  pros::delay(100);
  chassis.turnToHeading(130, 500, {.maxSpeed = 127}, false);
  lift.move(127);
  pros::delay(650);
  lift.brake();
  chassis.setPose(0, 0, 0);

  chassis.moveToPoint(0, -25, 1250, {.forwards = false, .minSpeed = 80}, false);
  lift.move(-127);
  pros::delay(525);
  lift.brake();
  claw_spin.move(127);
  pros::delay(200);
  lift.move(127);
  pros::delay(220);
  lift.brake();
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 15, 600, { .minSpeed = 80}, false);
  
  chassis.turnToHeading(-38, 500, {.maxSpeed = 127}, false);
chassis.setPose(0, 0, 0);
  lift.move(-127);
  pros::delay(400);
  lift.brake();
lift.move_absolute(0, 127);
 

spinclaw(2);



  chassis.moveToPoint(0, -35, 2450,
                      {.forwards = false, .maxSpeed = 30, .minSpeed = 10,
                       .earlyExitRange = 1},
                      true);
  chassis.waitUntil(27.5);
  spinclaw(0);
  pros::delay(700);
  spinclaw(1);

  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(130, 600, {.maxSpeed = 127}, false);
  lift.move(127);
  pros::delay(950);
  lift.brake();
  chassis.setPose(0, 0, 0);
  
  chassis.moveToPoint(0, -25, 1250, {.forwards = false, .minSpeed = 80}, false);
  lift.move(-127);
  pros::delay(525);
  lift.brake();
  claw_spin.move(127);
  pros::delay(200);
  lift.move(127);
  pros::delay(220);
  lift.brake();
}

} // namespace auton
