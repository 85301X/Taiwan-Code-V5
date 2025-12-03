#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
//#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/optical.hpp"
#include <cstdio>
#include <string>
#include "global.h"
#include "pros/rtos.hpp"
#include <iostream>
extern bool red_alliance;
void long_goal(int timeout){
    outpist.set_value(1);
    pros::delay(200);
    Stage_3.move_velocity(600);
    Stage_2.move_velocity(-600);
    pros::delay(timeout);
    Stage_3.move_velocity(0);
    Stage_2.move_velocity(0);
  
}
void timedriving(float timeout,bool forward,int velocity){
    if (forward){
        leftMotors.move_velocity(velocity);
        rightMotors.move_velocity(velocity);
        pros::delay(timeout);
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
    }
       else{
        leftMotors.move_velocity(-velocity);
        rightMotors.move_velocity(-velocity);
        pros::delay(timeout);
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
    }
}

void  middle_goal(int time) {
      Stage_3.move_velocity(-600);
                Stage_2.move_velocity(600);
    pros::delay(500);
        Stage_3.move_velocity(-600);
                Stage_2.move_velocity(-600);
 pros::delay(time);
        Stage_3.move_velocity(0);
        
                Stage_2.move_velocity(0);
    pros::delay(100);
    }

void intakemove(){
    intake.move_velocity(600);
    intakeon = true;
}
void pidtesting(){
    chassis.setPose(0,0,90);
    // a for loop to turn 0- degree to 360-degree 
    /*in increments of 90 degrees
   for (int nine = 0; nine < 360; nine += 90){
        chassis.turnToHeading(nine, 1000);
         pros::delay(1000);
    }
 */

   chassis.moveToPoint(12*3,0, 2000,{.decelStartDist=9,.decelFactor=1.9 });
 //  chassis.moveToPoint(0, 0, 2000,{.forwards=false,.decelStartDist=9,.decelFactor=1.9});
  

}

void Blue_Right_AWP() { // RED
    //=============set pose ============================================================
   /**
    chassis.setPose(0, 0, 270);
    intake.move_velocity(600);
    chassis.moveToPoint(-15.6, 0, 2000, {.maxSpeed = 127, .minSpeed = 80,.earlyExitRange=2,.decelStartDist=4,.decelFactor=1.5});
    load_1.set_value(true);
    chassis.waitUntilDone();
    chassis.swingToHeading(0,lemlib::DriveSide::RIGHT , 2000);
 

    chassis.moveToPoint(-32.5, 10, 1000, {.maxSpeed = 120, .minSpeed = 40});
    chassis.waitUntilDone();
     pros::delay(300);
 
    
   chassis.moveToPose(-33,-32,0, 1000, {.forwards = false,.lead=0.3, .maxSpeed = 127, .minSpeed = 80});
  

    chassis.waitUntilDone();

    //outpist.set_value(false);
    long_goal(2000);
    
    load_1.set_value(false);
       
    outpist.set_value(false);
    chassis.setPose(-35,-24,chassis.getPose().theta);

    chassis.moveToPoint(-35, -21, 100, {.forwards = true, .maxSpeed = 100, .minSpeed = 40,.earlyExitRange=2});
    chassis.turnToHeading(90,900);
 
    chassis.waitUntilDone();
    intake.move_velocity(600);
    Stage_3.move_velocity(600);
    Stage_2.move_velocity(-600);
    chassis.moveToPoint(-13, -27, 2000, {.maxSpeed = 127, .minSpeed = 80,.earlyExitRange=2});
    */
      chassis.setPose(-13, -27, chassis.getPose().theta);

      //=============set pose ============================================================
}



void soloAWP() { // RED
    //=============set pose ============================================================
    /*
    chassis.setPose(0, 0, 270);
    intake.move_velocity(600);
    chassis.moveToPoint(-19, 0, 2000, {.maxSpeed = 127, .minSpeed = 80,.earlyExitRange=2,.decelStartDist=4,.decelFactor=1.5});
    load_1.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 2000);
 
   
    chassis.moveToPoint(-34.5, 10, 1000, {.maxSpeed = 120, .minSpeed = 40});
    chassis.waitUntilDone();
     pros::delay(300);
 
    
   chassis.moveToPose(-35,-32,0, 1000, {.forwards = false,.lead=0.3, .maxSpeed = 127, .minSpeed = 40});
  
   
    chassis.waitUntilDone();

    //outpist.set_value(false);
    long_goal(2000);
    
    load_1.set_value(false);
      //=============set pose ============================================================
          
     
    outpist.set_value(false);
    chassis.setPose(-35,-24,chassis.getPose().theta);

    chassis.moveToPoint(-35, -21, 100, {.forwards = true, .maxSpeed = 100, .minSpeed = 40,.earlyExitRange=2});
    chassis.turnToHeading(90,900);
    chassis.waitUntilDone();
    intake.move_velocity(600);
    Stage_3.move_velocity(600);
    Stage_2.move_velocity(-600);
    chassis.moveToPoint(-13, -27, 2000, {.maxSpeed = 127, .minSpeed = 80,.earlyExitRange=2});
     
    chassis.moveToPoint(29, -27, 2000, {.maxSpeed = 127, .minSpeed = 70});
      pros::delay(60);
      load_1.set_value(true);
  
    //finish gettinng rings
    chassis.turnToHeading(46, 2000);
    chassis.moveToPoint(21, -44, 1000, {.forwards=false,.maxSpeed = 127, .minSpeed = 4});
    chassis.waitUntilDone();

    middle_goal(650);


    chassis.moveToPoint(54, -9, 2000, {.maxSpeed = 127, .minSpeed = 90});
     intake.move_velocity(600);
    
      pros::delay(300);
    
      load_1.set_value(true);
    chassis.swingToHeading(0 ,lemlib::DriveSide::LEFT, 500);

  
    chassis.waitUntilDone();
      chassis.moveToPoint(62.2, 8, 1000, {.maxSpeed = 127, .minSpeed = 90});
     
       chassis.waitUntilDone();
     pros::delay(200);
               
 
     Stage_2.move_velocity(0);
    chassis.moveToPoint(63.0, -24, 1000, {.forwards=false,.maxSpeed = 127, .minSpeed = 70});
    long_goal(2000);
    */
       
       /*
    chassis.turnToPoint(53,-5, 300,{.maxSpeed = 127, .minSpeed = 80,.earlyExitRange=2,});
  
    chassis.moveToPoint(53, -5, 2000, {.maxSpeed = 127, .minSpeed = 80,.earlyExitRange=2,.decelStartDist=4,.decelFactor=1.5});
    
    load_1.set_value(false);
    
    chassis.turnToHeading(0, 200);
    chassis.moveToPoint(67, -22, 2000, {.forwards=false,.maxSpeed = 127, .minSpeed = 40,.earlyExitRange=2,.decelStartDist=4,.decelFactor=1.5});
    chassis.waitUntilDone();
    long_goal(2000);
  */
    //outpist.set_value(0);
//     pros::delay(200);
//     pros::delay(300);
//     timedriving(300, true, 300);
//     outpist.set_value(false);
//     pros::delay(100);
//   //    chassis.moveToPoint(-35, -28,  2000,{.forwards=0,.maxSpeed = 120, .minSpeed = 30});
//     timedriving(300, false, 300);

}



void Auton_Skills_V4() {
  
    chassis.setPose(2.5, 1, 0);
    intake.move_velocity(600);
    chassis.moveToPoint(14, 30, 2000, {.maxSpeed = 50});
    chassis.waitUntilDone();
    // pros::delay(500);
    // chassis.turnToHeading(180, 2000);
    chassis.moveToPoint(29, -4.0, 2000, {.maxSpeed = 100});
    chassis.waitUntilDone();
    load_1.set_value(true);
    // chassis.turnToHeading(0, 2000);
    chassis.turnToHeading(180, 2000);
    chassis.waitUntilDone();
    chassis.setPose(29, -4, 180);

    // moving back into goal to score + align
    // chassis.moveToPoint(30, 16, 2250, {.forwards = false, .maxSpeed = 60, .minSpeed = 40});
    timedriving(1000, false,200);
    pros::delay(600);
    Stage_3.move_velocity(600);
    pros::delay(1500);
    Stage_3.move_velocity(0);
    chassis.waitUntilDone();
    chassis.setPose(29.5, 23, 180);

    // loader
   intake.move_velocity(600);
    chassis.moveToPoint(29.5, -17, 2000, {.maxSpeed = 80, .minSpeed = 40});
    // chassis.waitUntilDone();
    intake.move_velocity(600);
    chassis.waitUntilDone();
    pros::delay(1900);
    chassis.moveToPoint(29.5, 24, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40});
    chassis.waitUntilDone();
    
    // scoring again
    Stage_3.move_velocity(600);
    pros::delay(2000);
    Stage_3.move_velocity(0);
    chassis.setPose(35.5, 29, 180);

    // small push in
    load_1.set_value(false);
    timedriving(500, true,200);
    chassis.waitUntilDone();
    timedriving(700, false,200);
    chassis.waitUntilDone();
    Stage_3.move_velocity(600);
    lemlib::TrackingWheel vertical_tracking_wheel(&odomy, lemlib::Omniwheel::NEW_2, 3.65);
    chassis.setPose(29.5, 25, 180);
    

    // move to between loader and goal
    // chassis.moveToPose(36, 10, 180, 2000, {.lead = 0.1});
    Stage_3.move_velocity(0);
    intake.move_velocity(0);
    timedriving(700, true,200);
    // chassis.moveToPoint(36, 10, 2000, {.minSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(500);
    // chassis.turnToPoint(60, 23, 2000, {.minSpeed = 60});
    chassis.turnToHeading(45, 2000);
    // chassis.moveToPoint(43, 18, 2000, {.minSpeed = 40});
    chassis.waitUntilDone();
    chassis.moveToPoint(56.6, 22, 2000, {.minSpeed = 40});
    // chassis.moveToPose(56, 22, 45, 2000, {.lead = 0.1});
    chassis.waitUntilDone();
    intake.move_velocity(600);
    //===============================
    // rotating to other side
    chassis.turnToHeading(0, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(53, 103, 2000, {.maxSpeed = 127, .minSpeed = 40}); // 56.5 -> 55
    // chassis.turnToHeading(270, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(42.7, 92, 2000, {.forwards = false, .maxSpeed = 90}); // 75 -> 110 -> 92

    chassis.waitUntilDone();
    // chassis.moveToPoint(26.5, 95, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 2000);
    chassis.waitUntilDone();
    // chassis.moveToPoint(45, 82, 2000, {.forwards = false});

    // back into goal
    timedriving(600, false,200);
    
    chassis.setPose(44, 88, 0);
    pros::delay(500);

    // load
    load_1.set_value(true);
    chassis.moveToPoint(44, 121, 2000, {.maxSpeed = 65, .minSpeed = 40});
    chassis.waitUntilDone();
    pros::delay(2100);

    // score
    chassis.moveToPoint(44, 86, 2000, {.forwards = false, .maxSpeed = 100});


    chassis.waitUntilDone();
    pros::delay(600);
    Stage_3.move_velocity(600);
    intake.move_velocity(600);
    pros::delay(2500);
    Stage_3.move_velocity(0);
    //===============
    // move front to be ready to rotate to Q3
    // chassis.moveToPoint(44, 121, 2000, {.minSpeed = 40});
    timedriving(300, true,200);
    chassis.waitUntilDone();
    load_1.set_value(false);

    // rotate to Q3
    chassis.turnToHeading(270, 2000);
    chassis.waitUntilDone();
    // chassis.moveToPose(-62, 150, 0, 10000, {.lead = 0.1, .maxSpeed = 40});
    chassis.moveToPoint(-71+6.5, 115, 5000, {.maxSpeed =100});
    // position check
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 400);
     chassis.moveToPoint(-57.5+6.5, 60, 1800, {.forwards=false,.maxSpeed = 68, .minSpeed = 00});
     load_1.set_value(1);
    chassis.waitUntilDone();
    Stage_3.move_velocity(600);
    intake.move_velocity(600);
    pros::delay(1500);
    Stage_3.move_velocity(0);  

    // load
     chassis.moveToPoint(-57.5, 132, 2000, {.maxSpeed = 65, .minSpeed = 40});
     chassis.waitUntilDone();
     intake.move(600);
     pros:: delay(2700);
  
     // score
    chassis.moveToPoint(-57.5, 78,2000, {.forwards=false,.maxSpeed = 65, .minSpeed = 40});
    chassis.waitUntilDone();
    chassis.setPose(-58.5,78,0);
    Stage_3.move_velocity(600);
    intake.move_velocity(600);
    pros:: delay(2000);
    Stage_3.move_velocity(0);
    intake.move_velocity(0);

    // small push in
    load_1.set_value(false);
    timedriving(500, true,200);
    chassis.waitUntilDone();
    timedriving(700, false,200);
    chassis.waitUntilDone();
    Stage_3.move_velocity(600);
    
    chassis.setPose(-58.5, 78, 0);

    // rotating to Q4
    chassis.moveToPoint(-58.5, 93.6,2000); // 96.6 -> 98
    chassis.waitUntilDone();
    chassis.turnToHeading(50+180, 800);
    Stage_3.move_velocity(0);

    chassis.moveToPoint(-86.5, 68.6,2000,{});

    chassis.turnToHeading(180, 300);
    chassis.moveToPoint(-86.5, 3, 2000, {.maxSpeed = 100, .minSpeed = 40});

    // aligning with goal
    
    // chassis.moveToPoint(-76, 4, 2000, {.forwards = false});
     chassis.moveToPose(5, -27,90, 6000,{.lead =0.4,.minSpeed = 127,});
  
}


// AWPS

void Blue_Right_New_AWP() {
    intake.move_velocity(600);
    // set position to x:0, y:0, heading:0
    chassis.setPose(5, 7, 0);

    intake.move_velocity(400);
    chassis.moveToPose(16.8, 25.8, 360-315,2000,{.lead=0.6,.maxSpeed=60,.minSpeed=0,});

    chassis.turnToHeading(360-228+180,  400,{.minSpeed=20});
  // chassis.turnToPoint(25, 48, 300);
    chassis.waitUntilDone();
     timedriving(850, 1,200);
    
     pros::delay(200);
     
     intake.move_velocity(00);
     chassis.moveToPoint(23.5+18,10,2000, {.forwards=false,.minSpeed=0,.earlyExitRange=5});
      Stage_3.move_velocity(0);
     chassis.turnToHeading(180, 800);
    chassis.waitUntilDone();

    load_1.set_value(true);
    pros::delay(300);
     chassis.moveToPoint(20.5+12,-20,2000, {.forwards=true,.minSpeed=10});
 intake.move_velocity(600);
    
    chassis.waitUntilDone();
    pros::delay(300);

     chassis.moveToPoint(20.5+11+0.5,29,2000, {.forwards=false,.maxSpeed=60,.minSpeed=0,.earlyExitRange=5});
     chassis.waitUntilDone();
    intake.move_velocity(600);
    Stage_3.move_velocity(500);
    pros::delay(1500);
    Stage_3.move_velocity(0);
    timedriving(500, true,200);
    chassis.waitUntilDone();
    timedriving(700, false,500);
    chassis.waitUntilDone();
    Stage_3.move_velocity(600);

}

void Blue_New_Left_AWP() {
    chassis.setPose(-5,-2,0);
    pros::delay(1000);
    intake.move_velocity(600);
    chassis.moveToPose(-18.7, 27.8, 315, 1500, {.lead = 0.6, .maxSpeed = 75, .minSpeed = 0});
    
    chassis.waitUntilDone();
    // pros::delay(50);
    chassis.turnToHeading(180+45,  400,{.minSpeed=20});
    chassis.waitUntilDone();
    load_1.set_value(false);
    timedriving(520, 0,200);
    // timedriving(450, false);
    pros::delay(250);
    intake.move_velocity(600);
    Stage_3.move_velocity(200);
     pros::delay(600);
    // loader
    Stage_3.move_velocity(0);
    chassis.moveToPoint(-51.50,3.21,2000, {.forwards=true,.minSpeed=60,.earlyExitRange=5});
    chassis.turnToHeading(180, 400);
    chassis.waitUntilDone();
    load_1.set_value(true);
     pros::delay(200);

    Stage_3.move_velocity(0);
    intake.move_velocity(900);
    chassis.waitUntilDone();
    pros::delay(270);
    chassis.moveToPoint(-54-1.5,23,2000, {.forwards=false,.maxSpeed=127,.minSpeed=60,.earlyExitRange=5});
    chassis.waitUntilDone();
    load_1.set_value(false);
    intake.move_velocity(600);
    Stage_3.move_velocity(400);
    pros::delay(200);
    Stage_3.move_velocity(200);
    pros::delay(1500);
    Stage_3.move_velocity(0);
     // small push
    timedriving(500, true,200);
    chassis.waitUntilDone();
    timedriving(700, false,200);
    chassis.waitUntilDone();
    Stage_3.move_velocity(600);
}

void soloAmhWP() {
    // help
  
  
    chassis.setPose(-5,-2,0);
   // chassis.turnToHeading(90,1000);
   
    intake.move_velocity(600);

    chassis.moveToPose(-18.7, 27.8, 315, 1500, {.lead = 0.6, .maxSpeed = 75, .minSpeed = 0});
    // pros::delay(800);
    // load_1.set_value(true);
    chassis.waitUntilDone();
    // pros::delay(50);

    chassis.turnToHeading(180+43,  400,{.minSpeed=20});
    chassis.waitUntilDone();
    load_1.set_value(false);
     timedriving(500, 0,200);

    // timedriving(450, false);
         pros::delay(175);
        
    // scoring center goal
    intake.move_velocity(600);
    Stage_3.move_velocity(200);
     pros::delay(450);
    // loader
    Stage_3.move_velocity(0);
    //move to loader-------------------------------------------------
     chassis.moveToPoint(-51.50-2,3.21,2000, {.forwards=true,.minSpeed=60,.earlyExitRange=5});

    chassis.turnToHeading(180, 400);
    chassis.waitUntilDone();
    
    load_1.set_value(true);
     pros::delay(230);
    chassis.moveToPoint(-55.5-3,-19,700, {.forwards=true,.minSpeed=10}); // -23 -> -13 // 55.5 -> 52.5

    Stage_3.move_velocity(0);
    intake.move_velocity(900);
    chassis.waitUntilDone();
    pros::delay(300);
   
    chassis.moveToPoint(-54-3,23,2000, {.forwards=false,.maxSpeed=127,.minSpeed=60,.earlyExitRange=5});
    chassis.waitUntilDone();
    load_1.set_value(false);
    intake.move_velocity(600);
    Stage_3.move_velocity(400);
    pros::delay(200);
    Stage_3.move_velocity(200);
    pros::delay(610);

    Stage_3.move_velocity(0);
    
    timedriving(500, true,200);
    chassis.waitUntilDone();
    timedriving(700, false,200);
    chassis.waitUntilDone();

   chassis.moveToPoint(46.5-2-8+9,23-7-6-16,7000, {.forwards=true,.maxSpeed=127,.minSpeed=20,.earlyExitRange=5});
   chassis.turnToHeading(180, 500);
      chassis.waitUntilDone();
     intake.move_velocity(600);
    timedriving(900, false,400);
      Stage_3.move_velocity(600);
          timedriving(500, true,200);

    chassis.setPose(0,0,270);
    chassis.moveToPoint(-33, 0, 1500);
    chassis.turnToHeading(180, 5000);
    chassis.waitUntilDone();
    load_1.set_value(true);
    intake.move_velocity(600);
    Stage_3.move_velocity(600);
    Stage_2.move_velocity(-600);
    pros::delay(1000);
    chassis.moveToPoint(-31.5, -12, 2000);
 
}

void Auton_Skills(){
    Auton_Skills_V4();


}