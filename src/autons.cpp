#include "autons.hpp"
#include "lemlib/chassis/chassis.hpp"
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
  chassis.moveToPoint(0, -5, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 4, 450, {.minSpeed = 100}, false);
  chassis.moveToPoint(0, -5, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 4, 450, {.minSpeed = 100}, false);
  spinclaw(1);
  chassis.moveToPoint(-24, -9.8, 1300, {.forwards = false, .minSpeed = 100},
                      false);
  claw_spin.move(127);
  pros::delay(650);
  
  spinclaw(2);
   pros::delay(100);
  chassis.setPose(0,0,0);
  pros::delay(50);
  claw_spin.move(-127);
  chassis.moveToPoint(0, 10.3, 650, {.maxSpeed = 100}, false);
  chassis.turnToHeading(-139, 300, {.maxSpeed = 127}, false);
  chassis.setPose(-5, 0, 0);
  // exits 1 in short so the turn does not spike at the point
  chassis.moveToPoint(0, -15, 1410,
                      {.forwards = false, .maxSpeed = 30, .minSpeed = 10,
                       .earlyExitRange = 1},
                      true);
  chassis.waitUntil(12);
  spinclaw(0);
  pros::delay(700);
  spinclaw(1);

  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
  lift.move_absolute(750, 127);
pros::delay(150);
  chassis.turnToHeading(127, 500, {.maxSpeed = 127}, false);

  chassis.setPose(0, 0, 0);

  chassis.moveToPoint(0, -25, 1250, {.forwards = false, .maxSpeed = 90}, false);
  lift.move_absolute(300, 127);

  claw_spin.move(127);
  pros::delay(200);

lift.move_absolute(900, 127);
pros::delay(800);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 24, 600, { .minSpeed = 80}, false);
  
  chassis.turnToHeading(-39.3, 500, {.maxSpeed = 127}, false);
chassis.setPose(0, 0, 0);
  lift.move(-127);
  pros::delay(400);
  lift.brake();
 

spinclaw(2);


claw_spin.move(-127);
  chassis.moveToPoint(0, -35, 2380,
                      {.forwards = false, .maxSpeed = 60, .minSpeed = 10,
                       .earlyExitRange = 1},
                      true);
  chassis.waitUntil(27.5);
  spinclaw(0);
  pros::delay(700);
  spinclaw(1);

  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
   lift.move_absolute(1200, 127);
  pros::delay(300);
  chassis.turnToHeading(130, 600, {.maxSpeed = 127}, false);
 
  chassis.setPose(0, 0, 0);
  
  chassis.moveToPoint(0, -25, 1250, {.forwards = false, .minSpeed = 80}, false);
  lift.move_absolute(700,127);
  claw_spin.move(127);
  pros::delay(200);
    lift.move_absolute(1200,127);



   chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(127);
    chassis.turnToHeading(32, 600, {.maxSpeed = 127}, false);



   chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
spinclaw(2);
  
  chassis.moveToPoint(6, 25.5, 2272, {}, false);
  lift.move_absolute(0, 127);
    spinclaw(0);


  chassis.turnToHeading(90, 600, {}, false);

   chassis.moveToPoint(25, 25.5, 1972, {}, false);
   chassis.waitUntil(14);
   
    intake.move(127);
  
       chassis.moveToPoint(24, 25.5, 1572, {}, false);
       pros::delay(500);
       
  chassis.moveToPoint(6, 25.5, 1572, {.forwards = false}, false);
   spinclaw(1);
  lift.move_absolute(2600, 127);
  pros::delay(200);
  chassis.turnToHeading(-6, 600, {}, false);
 
            chassis.moveToPoint(0, -3, 1772, {.forwards = false, .maxSpeed = 80}, false);
             chassis.turnToHeading(0, 50, {}, false);
             lift.move_absolute(1800, 127);
             claw_spin.move(127);
             pros::delay(700);
             
            
               lift.move_absolute(2600, 127);
               claw_spin.move(-127);
               spinclaw(2);

               //end last goal 1 score



   



               
  chassis.moveToPoint(6, 28.1, 1572, {}, false);
  lift.move_absolute(0, 127);
    spinclaw(0);
  chassis.turnToHeading(90, 600, {}, false);

   chassis.moveToPoint(23, 28.1, 1572, {}, false);
   chassis.waitUntil(14);
     chassis.turnToHeading(92, 50, {}, false);
    chassis.turnToHeading(88, 50, {}, false);
       chassis.moveToPoint(22, 28.1, 1572, {}, false);
       pros::delay(900);
lift.move_absolute(0, 127);


       chassis.setPose(0,0,0);
           chassis.turnToPoint(-9,35,600,{}, false);
           spinclaw(1);
           chassis.moveToPoint(9, -35, 2572, {.forwards = false, .maxSpeed = 80}, false);
           chassis.waitUntil(20);
            
            
           
             
              claw_spin.move(127);
              pros::delay(200);
              spinclaw(2);
              chassis.moveToPoint(0, -10, 1772, {}, false);
              spinclaw(0);
                chassis.turnToHeading(0, 100, {}, false);

//goal 2 first score
                
   chassis.moveToPoint(4, 2, 1572, {}, false);
  
   chassis.waitUntil(4);
     chassis.turnToHeading(2, 50, {}, false);
    chassis.turnToHeading(-2, 50, {}, false);
       chassis.moveToPoint(0, -1, 1572, {}, false);
       chassis.setPose(0,0,0);
       pros::delay(1100);
lift.move_absolute(0, 127);






          chassis.turnToPoint(-9,35,600,{}, false);
       spinclaw(1);
         lift.move_absolute(800,127);
           chassis.moveToPoint(9, -35, 2572, {.forwards = false, .maxSpeed = 70}, false);
           chassis.waitUntil(20);
             
         
          
            
            
              lift.move_absolute(300, 127);
              claw_spin.move(127);
              pros::delay(800);
              lift.move_absolute(800,127);
              spinclaw(2);
              chassis.moveToPoint(-3, -9, 1772, {}, false);
              spinclaw(0);
                chassis.turnToHeading(0, 100, {}, false);
lift.move_absolute(0,127);

//end goal 2 second score

                
   chassis.moveToPoint(0, 0, 1572, {}, false);
   chassis.waitUntil(4);
     chassis.turnToHeading(2, 50, {}, false);
    chassis.turnToHeading(-2, 50, {}, false);
       chassis.moveToPoint(0, -1, 1572, {}, false);
       pros::delay(1500);
lift.move_absolute(0, 127);
    chassis.turnToPoint(-9,35,600,{}, false);
  spinclaw(1);
     lift.move_absolute(1400,127);
     pros::delay(400);
           chassis.moveToPoint(9, -35, 2572, {.forwards = false, .maxSpeed = 70}, false);
           chassis.waitUntil(20);
        
           
            
            
              lift.move_absolute(950, 127);
              claw_spin.move(127);
              pros::delay(800);
              lift.move_absolute(1400,127);
              spinclaw(2);
              chassis.moveToPoint(-3, -9, 1772, {}, false);
             spinclaw(0);
              chassis.turnToHeading(0, 100, {}, false);
lift.move_absolute(0,127);







                
   chassis.moveToPoint(0, 0, 1572, {}, false);
   chassis.waitUntil(4);
     chassis.turnToHeading(2, 50, {}, false);
    chassis.turnToHeading(-2, 50, {}, false);
       chassis.moveToPoint(0, -1, 1572, {}, false);
       pros::delay(1500);
lift.move_absolute(0, 127);
    chassis.turnToPoint(-9,35,600,{}, false);
  spinclaw(1);
     lift.move_absolute(1900,127);
           chassis.moveToPoint(9, -35, 2572, {.forwards = false, .maxSpeed = 70}, false);
           chassis.waitUntil(20);
        
           
            
            
              lift.move_absolute(1400, 127);
              pros::delay(400);
              claw_spin.move(127);
              pros::delay(800);
              lift.move_absolute(1900,127);
              spinclaw(2);
              chassis.moveToPoint(-3,9, 1772, {}, false);
             spinclaw(0);
              chassis.turnToHeading(0, 100, {}, false);
lift.move_absolute(0,127);
  /*
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


// 58_skills
void skills58() {

  chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
  chassis.moveToPoint(0, -5, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 4, 450, {.minSpeed = 100}, false);
  chassis.moveToPoint(0, -5, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 4, 450, {.minSpeed = 100}, false);
  spinclaw(1);
  chassis.moveToPoint(-24, -9.8, 1300, {.forwards = false, .minSpeed = 100},
                      false);
  claw_spin.move(127);
  pros::delay(650);
  
  spinclaw(2);
   pros::delay(100);
  chassis.setPose(0,0,0);
  pros::delay(50);
  claw_spin.move(-127);
  chassis.moveToPoint(0, 10.3, 650, {.maxSpeed = 100}, false);
  chassis.turnToHeading(-139, 300, {.maxSpeed = 127}, false);
  chassis.setPose(-5, 0, 0);
  // exits 1 in short so the turn does not spike at the point
  chassis.moveToPoint(0, -15, 1410,
                      {.forwards = false, .maxSpeed = 30, .minSpeed = 10,
                       .earlyExitRange = 1},
                      true);
  chassis.waitUntil(12);
  spinclaw(0);
  pros::delay(700);
  spinclaw(1);

  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
  lift.move_absolute(750, 127);
pros::delay(150);
  chassis.turnToHeading(127, 500, {.maxSpeed = 127}, false);

  chassis.setPose(0, 0, 0);

  chassis.moveToPoint(0, -25, 1250, {.forwards = false, .maxSpeed = 90}, false);
  lift.move_absolute(300, 127);

  claw_spin.move(127);
  pros::delay(200);

lift.move_absolute(900, 127);
pros::delay(800);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 24, 600, { .minSpeed = 80}, false);
  
  chassis.turnToHeading(-39.3, 500, {.maxSpeed = 127}, false);
chassis.setPose(0, 0, 0);
  lift.move(-127);
  pros::delay(400);
  lift.brake();
 

spinclaw(2);


claw_spin.move(-127);
  chassis.moveToPoint(0, -35, 2380,
                      {.forwards = false, .maxSpeed = 60, .minSpeed = 10,
                       .earlyExitRange = 1},
                      true);
  chassis.waitUntil(27.5);
  spinclaw(0);
  pros::delay(700);
  spinclaw(1);

  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
   lift.move_absolute(1200, 127);
  pros::delay(300);
  chassis.turnToHeading(130, 600, {.maxSpeed = 127}, false);
 
  chassis.setPose(0, 0, 0);
  
  chassis.moveToPoint(0, -25, 1250, {.forwards = false, .minSpeed = 80}, false);
  lift.move_absolute(700,127);
  claw_spin.move(127);
  pros::delay(200);
    lift.move_absolute(1200,127);



   chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(127);
    chassis.turnToHeading(20, 600, {.maxSpeed = 127}, false);
  //yadda yadda
    chassis.turnToHeading(45, 600, {.maxSpeed = 127}, false);



   chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
spinclaw(2);
  



  chassis.moveToPoint(0, 15, 2272, {}, false);
  
  chassis.turnToHeading(180, 900, {}, false);
  
  lift.move_absolute(0, 127);
    spinclaw(0);
  chassis.moveToPoint(0, 55, 2272, {.forwards = false}, false);
chassis.setPose(0,30,180);



   chassis.moveToPoint(0, 25.7, 1972, {}, false);



  chassis.turnToHeading(75, 600, {}, false);

   chassis.moveToPoint(26, 25.7, 1972, {}, false);
   chassis.waitUntil(14);
   
    intake.move(127);
  
       chassis.moveToPoint(24, 25.7, 1572, {}, false);
       pros::delay(500);
       
  chassis.moveToPoint(6, 26.2, 1572, {.forwards = false}, false);
   spinclaw(1);
  lift.move_absolute(2600, 127);
  pros::delay(200);
  chassis.turnToHeading(0, 600, {}, false);
 
            chassis.moveToPoint(0, -1, 2972, {.forwards = false, .maxSpeed = 60}, false);
             chassis.turnToHeading(0, 50, {}, false);
             lift.move_absolute(1800, 127);
             claw_spin.move(127);
             pros::delay(700);
             
            
               lift.move_absolute(2600, 127);
               claw_spin.move(-222127);
               spinclaw(2);
               chassis.setPose(0,0,0);
                   chassis.turnToHeading(32, 150, {}, false);
               //end last goal 1 score
        chassis.setPose(0,0,0);


   



               
  chassis.moveToPoint(6, 27.5, 1572, {}, false);
  lift.move_absolute(0, 127);
    spinclaw(0);
  chassis.turnToHeading(90, 600, {}, false);

   chassis.moveToPoint(27, 28.1, 1572, {}, false);
   chassis.waitUntil(14);
     chassis.turnToHeading(92, 50, {}, false);
    chassis.turnToHeading(88, 50, {}, false);
       chassis.moveToPoint(22, 28.1, 1572, {}, false);
       pros::delay(900);
lift.move_absolute(0, 127);


       chassis.setPose(0,0,0);
           chassis.turnToPoint(-9,35,600,{}, false);
           spinclaw(1);
           chassis.moveToPoint(9, -35, 2572, {.forwards = false, .maxSpeed = 80}, false);
           chassis.waitUntil(20);
            
            
           
             
              claw_spin.move(127);
              pros::delay(200);
              spinclaw(2);
              chassis.moveToPoint(0, -10, 1772, {}, false);
              spinclaw(0);
                chassis.turnToHeading(0, 100, {}, false);
                claw_spin.move(-127);

//goal 2 first score

chassis.moveToPoint(0, 0, 1772, {}, false);
intake.move(127);
pros::delay(600);

chassis.moveToPoint(0, -7, 1772, {.forwards = false}, false);
spinclaw(1);
chassis.turnToPoint(-72, 72, 1100, {}, false);
chassis.moveToPoint(72, -72, 3772, {.forwards = false}, false);
lift.move_absolute(1300, 127);
pros::delay(100);
chassis.moveToPoint(0, -7, 3772, {.forwards = false}, false);
lift.move_absolute(600, 127);
pros::delay(400);
chassis.turnToHeading(-90, 600, {}, false);
chassis.setPose(0,0,0);
chassis.moveToPoint(0, 68, 2772, {}, false);
chassis.setPose(0,0,0);
  chassis.moveToPoint(0, 6, 450, {.minSpeed = 100}, false);
  chassis.moveToPoint(0, 2, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 6, 450, {.minSpeed = 100}, false);
  chassis.moveToPoint(0, 2, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, -68, 2872, {.forwards = false}, false);
}

void twenty_pts() {
  

  chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
  chassis.moveToPoint(0, -5, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 4, 450, {.minSpeed = 100}, false);
  chassis.moveToPoint(0, -5, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 4, 450, {.minSpeed = 100}, false);
  spinclaw(1);
  chassis.moveToPoint(24, -9.8, 1300, {.forwards = false, .minSpeed = 100},
                      false);
  claw_spin.move(127);
  pros::delay(550);
  
  spinclaw(2);
  
  chassis.moveToPoint(0, -24, 900, { .minSpeed = 100},
                      false);
  claw_spin.move(-127);
    chassis.moveToPoint(-19, -43, 2000, {.forwards = false, .maxSpeed = 60, .earlyExitRange = 1},
                      false);
                      
  chassis.waitUntil(13);
  spinclaw(0);
  pros::delay(700);
  spinclaw(1);
  chassis.waitUntilDone();
  lift.move_absolute(1000, 127);
pros::delay(300);
    chassis.moveToPoint(-24, -31, 1800, {.forwards = false, .maxSpeed = 110, .earlyExitRange = 1},
                      false);
       
     lift.move_absolute(700, 127);
                    claw_spin.move(127);
                    pros::delay(300);
                    
  lift.move_absolute(1100, 127);
  pros::delay(300);
  chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 6, 180, {},
                      false);
                        lift.move_absolute(0, 127);
                         pros::delay(300);
                         claw_spin.move(-127);
                         spinclaw(2);


}

// 30pts -- scoring routine. Empty on purpose: the motions have not been
// decided yet, and a half-guessed auton is worse than one that does nothing.
void thirty_pts() {

  
  chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(-127);
  chassis.moveToPoint(0, -5, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 4, 450, {.minSpeed = 100}, false);
  chassis.moveToPoint(0, -5, 400, {.forwards = false, .minSpeed = 70}, false);
  chassis.moveToPoint(0, 4, 450, {.minSpeed = 100}, false);
  spinclaw(1);
  chassis.moveToPoint(-24, -9.8, 1300, {.forwards = false, .minSpeed = 100},
                      false);
  claw_spin.move(127);
  pros::delay(650);
  
  spinclaw(2);
   pros::delay(100);
  chassis.setPose(0,0,0);
  pros::delay(50);
  claw_spin.move(-127);
  chassis.moveToPoint(0, 10.3, 650, {.maxSpeed = 100}, false);
  chassis.turnToHeading(-139, 300, {.maxSpeed = 127}, false);
  chassis.setPose(-5, 0, 0);
  // exits 1 in short so the turn does not spike at the point
  chassis.moveToPoint(0, -15, 1410,
                      {.forwards = false, .maxSpeed = 30, .minSpeed = 10,
                       .earlyExitRange = 1},
                      true);
  chassis.waitUntil(12);
  spinclaw(0);
  pros::delay(700);
  spinclaw(1);

  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
  lift.move_absolute(750, 127);
pros::delay(150);
  chassis.turnToHeading(127, 500, {.maxSpeed = 127}, false);

  chassis.setPose(0, 0, 0);

  chassis.moveToPoint(0, -25, 1250, {.forwards = false, .maxSpeed = 90}, false);
  lift.move_absolute(300, 127);

  claw_spin.move(127);
  pros::delay(200);

lift.move_absolute(900, 127);
pros::delay(800);
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(0, 24, 600, { .minSpeed = 80}, false);
  
  chassis.turnToHeading(-39.3, 500, {.maxSpeed = 127}, false);
chassis.setPose(0, 0, 0);
  lift.move(-127);
  pros::delay(400);
  lift.brake();
 

spinclaw(2);


claw_spin.move(-127);
  chassis.moveToPoint(0, -35, 2380,
                      {.forwards = false, .maxSpeed = 60, .minSpeed = 10,
                       .earlyExitRange = 1},
                      true);
  chassis.waitUntil(27.5);
  spinclaw(0);
  pros::delay(700);
  spinclaw(1);

  chassis.waitUntilDone();
  chassis.setPose(0, 0, 0);
   lift.move_absolute(1300, 127);
  pros::delay(500);
  chassis.turnToHeading(130, 600, {.maxSpeed = 127}, false);
 
  chassis.setPose(0, 0, 0);
  
  chassis.moveToPoint(0, -25, 1250, {.forwards = false, .minSpeed = 80}, false);
  lift.move_absolute(950,127);
  claw_spin.move(127);
  pros::delay(200);
    lift.move_absolute(1600,127);
pros::delay(500);


   chassis.setPose(0, 0, 0);
  intake.move(127);
  claw_spin.move(127);
    chassis.turnToHeading(20, 600, {.maxSpeed = 127}, false);
}

} // namespace auton
