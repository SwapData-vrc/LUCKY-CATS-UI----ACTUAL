#include "autons.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

#include <cmath>
#include <cstdio>

namespace auton {

// skills -- designed in Catpath.
// red alliance. Absolute field coordinates, inches from centre,
// LemLib headings. Nothing here mirrors: export again from the other side.
/*
void points(){
chassis.setPose(0, 0, 0);
intake.move(127);
claw_spin.move(-127);



}
*/
void skills() {
  chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
  spinclaw(1);
    chassis.moveToPoint(0, -2.3, 552, {.forwards = false, .maxSpeed = 70}, false);
   chassis.turnToHeading(-90, 700, {.maxSpeed = 80}, false);
    chassis.moveToPoint(10, -2.3, 852, {.forwards = false, .maxSpeed = 70}, false);
    claw_spin.move(127);
    pros::delay(1000);
    spinclaw(2);
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(5, 100, {.maxSpeed = 80}, false);
        chassis.setPose(0, 0, 0);
        
        intake.move(-127);
         chassis.moveToPoint(0, 23, 968, { .maxSpeed = 70}, false);
         spinclaw(0);
        chassis.turnToHeading(180, 2000, {.maxSpeed = 80}, false);
         chassis.moveToPoint(0, 60, 1572, {.forwards = false, .maxSpeed = 80}, false);
         chassis.setPose(0, 0, 0);
         intake.move(127);
         claw_spin.move(-127);
         chassis.moveToPoint(0, 0, 572, { .maxSpeed = 80}, false);
         chassis.turnToHeading(-130, 800, {.maxSpeed = 80}, false);
           chassis.moveToPoint(-30, -3, 3572, { .maxSpeed = 80}, false);
           chassis.setPose(0, 0, 0);
           chassis.moveToPoint(0,-0.5, 100, {.forwards = false, .maxSpeed = 80}, false);
           pros::delay(500);
            chassis.moveToPoint(0, -2.3, 552, {.forwards = false, .maxSpeed = 70}, false);
   chassis.turnToHeading(-90, 700, {.maxSpeed = 80}, false);
   spinclaw(1); 
  lift.move_absolute(900, 127);
  pros::delay(100);
    chassis.moveToPoint(20, -2.3, 1852, {.forwards = false, .maxSpeed = 70}, false);
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
         chassis.moveToPoint(0, 23, 968, { .maxSpeed = 70}, false);
         spinclaw(0);
         lift.move_absolute(-20, 127);
        chassis.turnToHeading(180, 2000, {.maxSpeed = 80}, false);
         chassis.moveToPoint(0, 60, 1572, {.forwards = false, .maxSpeed = 80}, false);
         chassis.setPose(0, 0, 0);
         intake.move(127);
         claw_spin.move(-127);
         chassis.moveToPoint(0, 0, 572, { .maxSpeed = 80}, false);
         chassis.turnToHeading(-130, 800, {.maxSpeed = 80}, false);
           chassis.moveToPoint(-30, -3, 3572, { .maxSpeed = 80}, false);
           chassis.setPose(0, 0, 0);
           chassis.moveToPoint(0,-0.5, 100, {.forwards = false, .maxSpeed = 80}, false);
           pros::delay(500);
            chassis.moveToPoint(0, -2.3, 552, {.forwards = false, .maxSpeed = 70}, false);
   chassis.turnToHeading(-90, 700, {.maxSpeed = 80}, false);
   spinclaw(1); 
  lift.move_absolute(1500, 127);
  pros::delay(100);
    chassis.moveToPoint(20, -2.3, 1852, {.forwards = false, .maxSpeed = 70}, false);
    lift.move_absolute(800, 127);
    pros::delay(100);
    claw_spin.move(127);
    pros::delay(10);
    lift.move_absolute(1500, 127);
 pros::delay(100);





 
chassis.setPose(0, 0, 0);
           
        intake.move(-127);
         chassis.moveToPoint(0, 23, 968, { .maxSpeed = 70}, false);
         spinclaw(0);
         lift.move_absolute(-20, 127);
        chassis.turnToHeading(180, 2000, {.maxSpeed = 80}, false);
         chassis.moveToPoint(0, 60, 1572, {.forwards = false, .maxSpeed = 80}, false);
         chassis.setPose(0, 0, 0);
         intake.move(127);
         claw_spin.move(-127);
         chassis.moveToPoint(0, 0, 572, { .maxSpeed = 80}, false);
         chassis.turnToHeading(-130, 800, {.maxSpeed = 80}, false);
           chassis.moveToPoint(-30, -3, 3572, { .maxSpeed = 80}, false);
           chassis.setPose(0, 0, 0);
           chassis.moveToPoint(0,-0.5, 100, {.forwards = false, .maxSpeed = 80}, false);
           pros::delay(500);
            chassis.moveToPoint(0, -2.3, 552, {.forwards = false, .maxSpeed = 70}, false);
   chassis.turnToHeading(-90, 700, {.maxSpeed = 80}, false);
   spinclaw(1); 
  lift.move_absolute(1700, 127);
  pros::delay(100);
    chassis.moveToPoint(20, -2.3, 1852, {.forwards = false, .maxSpeed = 70}, false);
    lift.move_absolute(1000, 127);
    pros::delay(100);
    claw_spin.move(127);
    pros::delay(10);
    lift.move_absolute(1700, 127);
 pros::delay(100);


 
chassis.setPose(0, 0, 0);
           
        intake.move(-127);
         chassis.moveToPoint(0, 23, 968, { .maxSpeed = 70}, false);
         spinclaw(0);
         lift.move_absolute(-20, 127);
        chassis.turnToHeading(180, 2000, {.maxSpeed = 80}, false);
         chassis.moveToPoint(0, 60, 1572, {.forwards = false, .maxSpeed = 80}, false);
         chassis.setPose(0, 0, 0);
         intake.move(127);
         claw_spin.move(-127);
         chassis.moveToPoint(0, 0, 572, { .maxSpeed = 80}, false);
         chassis.turnToHeading(-130, 800, {.maxSpeed = 80}, false);
           chassis.moveToPoint(-30, -3, 3572, { .maxSpeed = 80}, false);
           chassis.setPose(0, 0, 0);
           chassis.moveToPoint(0,-0.5, 100, {.forwards = false, .maxSpeed = 80}, false);
           pros::delay(500);
            chassis.moveToPoint(0, -2.3, 552, {.forwards = false, .maxSpeed = 70}, false);
   chassis.turnToHeading(-90, 700, {.maxSpeed = 80}, false);
   spinclaw(1); 
  lift.move_absolute(2100, 127);
  pros::delay(100);
    chassis.moveToPoint(20, -2.3, 1852, {.forwards = false, .maxSpeed = 70}, false);
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

  chassis.moveToPoint(0, -34, 3104, {.forwards = false, .maxSpeed = 23.9}, false);
  chassis.waitUntil(-26);
  lift.move_absolute(0, 70);
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
  chassis.moveToPoint(0, -24, 1000, {.forwards = false, .maxSpeed = 80}, false);
  lift.move_absolute(100, 127);
pros::delay(40);
  claw_spin.move(127);
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
  chassis.moveToPoint(0, -27, 1000, {.forwards = false, .maxSpeed = 50}, false);
  lift.move_absolute(1300, 127);

  claw_spin.move(-127);
  pros::delay(1000);
  lift.move_absolute(400, 127);
  pros::delay(100);
  claw_spin.move(127);
  lift.move_absolute(1500, 127);
  pros::delay(100);
  */
}

} // namespace auton


//pros mu --debug flag