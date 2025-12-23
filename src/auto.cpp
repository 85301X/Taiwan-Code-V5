#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/display/lv_display.h"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
//#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
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
#include "pros/screen.h"
#include <iostream>


double leftWallXFromDistance(



) {
   double leftDistanceInches = leftdist.get_distance() / 25.4; // Convert to inches
    // Perpendicularity factor (0 = good, 1 = bad)
    double angleFactor = fabs(sin(chassis.getPose().theta * (M_PI / 180.0)));

    // Reduce influence as angle worsens
    double correctionScale = 1.0 - angleFactor;


    // Final corrected X (no dependence on previous pose)
    return leftDistanceInches * correctionScale;
}

void checkautonrun(){
  if (   skills_auton_running == false){
    opcontrol();
  }
  else{
    pros::delay(10);
  }
}

void long_goal(int timeout){
  //outpist turn on
    outpist.set_value(1);

    pros::delay(50);

    //spin intake in long goal
    Stage_3.move_velocity(600);
    Stage_2.move_velocity(-600);
    
    pros::delay(timeout);
    

    // intake top 
    Stage_3.move_velocity(0);
    Stage_2.move_velocity(0);

}



void outaking(int timeout){

  //reverse intake
  intake.move_velocity(-600);
  Stage_3.move_velocity(-600);
  Stage_2.move_velocity(600);

  pros::delay(timeout);

  Stage_3.move_velocity(0);
  Stage_2.move_velocity(0);
  intake.move_velocity(-0);
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

void  middle_goal(int velocity,int time) {

    //reverse intake first
    intake.move_velocity(-600);
    Stage_3.move_velocity(-600);
    Stage_2.move_velocity(600);

    pros::delay(200);

    //middle goal
    Stage_3.move_velocity(-velocity);
    Stage_2.move_velocity(-velocity);
    intake.move_velocity(600);
    pros::delay(time);
    Stage_2.move_velocity(0);


    }

    void  skills_middle_goal(int velocity,int time) {

    //reverse intake first
    intake.move_velocity(-200);
    Stage_3.move_velocity(-600);
    Stage_2.move_velocity(600);

    pros::delay(200);

    //middle goal
    Stage_3.move_velocity(-600);
    Stage_2.move_velocity(-600);
    intake.move_velocity(600);
    pros::delay(600);
     Stage_3.move_velocity(-400);
    Stage_2.move_velocity(-400);
    intake.move_velocity(400);
    pros::delay(400);
      Stage_3.move_velocity(-600);
    Stage_2.move_velocity(-600);
    intake.move_velocity(600);
    pros::delay(200);
    Stage_2.move_velocity(0);


    }


void pidtesting(){
    chassis.setPose(0,0,90);
  skills_middle_goal(200, 1600);
    // a for loop to turn 0- degree to 360-degree 
    //in increments of 90 degrees
  /** 
   for (int nine = 0; nine < 360; nine += 90){
        chassis.turnToHeading(nine, 1000);
         pros::delay(1000);
    }
 */

  

}

/**==========================
++------------------------------++
++------------------------------++
||    _         _               ||
||   / \  _   _| |_ ___  _ __   ||
||  / _ \| | | | __/ _ \| '_ \  ||
|| / ___ \ |_| | || (_) | | | | ||
||/_/   \_\__,_|\__\___/|_| |_| ||
||                              ||
++------------------------------++
++------------------------------++

*/

/*
++------------------------------------------++
++------------------------------------------++
|| _                                      _ ||
||| |    _____      __   __ _  ___   __ _| |||
||| |   / _ \ \ /\ / /  / _` |/ _ \ / _` | |||
||| |__| (_) \ V  V /  | (_| | (_) | (_| | |||
|||_____\___/ \_/\_/    \__, |\___/ \__,_|_|||
||                      |___/               ||
++------------------------------------------++
++------------------------------------------++

*/
void not_work(){
chassis.setPose(0, 0, 0);
  intake.move_velocity(600);
    Stage_2.move_velocity(400);
  chassis.turnToHeading(360-345, 300);
  chassis.moveToPoint(5,19, 1000,{.maxSpeed = 127, .minSpeed = 90,.earlyExitRange=2});
  pros::delay(30);
   load_1.set_value(true);
   chassis.moveToPoint(10,34, 2000,{.maxSpeed = 127, .minSpeed = 100,.earlyExitRange=2});
   load_1.set_value(false);
   chassis.swingToHeading(90,DriveSide::RIGHT, 1000,{.maxSpeed = 127, .minSpeed = 100,.earlyExitRange=2});
    chassis.moveToPoint(23,39+6.5, 2000,{.maxSpeed = 127, .minSpeed = 100,.useDistSensor=true,.distanceSenseTarget=22 });
    chassis.waitUntilDone();
    
       pros::delay(900);
       chassis.swingToHeading(360-345,DriveSide::RIGHT, 1000,{.maxSpeed = 127, .minSpeed = 100,.earlyExitRange=2});
        chassis.moveToPoint(11.4,25, 1000,{.forwards=false,.maxSpeed = 127, .minSpeed = 90});
          chassis.turnToPoint(-4.2,36.9, 1000,{.maxSpeed = 127, .minSpeed = 40,.earlyExitRange=2});
   
         chassis.moveToPoint(-4.2,36.9, 1000,{.maxSpeed = 127, .minSpeed = 40,.earlyExitRange=2});
                pros::delay(1000);
            chassis.moveToPoint(29.9,8.9, 1000,{.forwards=false,.maxSpeed = 127, .minSpeed = 40,.earlyExitRange=2});
     chassis.turnToHeading(180, 1000);
     chassis.waitUntilDone();
     chassis.setPose(leftdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
     chassis.moveToPoint(12.9,3, 1000,{.maxSpeed = 127, .minSpeed = 40,.earlyExitRange=2});

       pros::delay(900);
          chassis.moveToPoint(12.9,40, 1000,{.forwards=false,.maxSpeed = 127, .minSpeed = 40});

}
 
void Low_Goal() { // RED
  
  
    
    //=============set pose ============================================================
    //Step 1 :: Aim the loader

    chassis.setPose(0, 0, 90);
    
    chassis.moveToPoint(27-0.37, 0, 800, {.maxSpeed = 127, .minSpeed = 60});
      chassis.waitUntilDone();
    chassis.setPose(frontdist.get_distance()/25.4,chassis.getPose().y,chassis.getPose().theta);
        pros::delay(20);
    load_1.set_value(true);
    chassis.turnToHeading(180, 1000,{.maxSpeed = 127, .minSpeed = 30,});
   
    intake.move_velocity(400);
    Stage_2.move_velocity(-600);
    Stage_3.move_velocity(600);


    //=============set pose ============================================================
    //Step 2 ::Match load and aim to goal
  
       

  int t_x=leftdist.get_distance()/25.4;
   chassis.moveToPoint(33.9-2-0.4+0.9, -13, 1000, {.maxSpeed = 90, .minSpeed = 48,});
    pros::delay(300);
   chassis.moveToPoint(33-1-+0.33+1.3,32, 1200, {.forwards = false, .maxSpeed = 127, .minSpeed = 60});
    
    long_goal(2000);
    load_1.set_value(false);
       
    

  //=============set pose ============================================================
  //Step 3 : aim low goal

    chassis.setPose(36,24,180);

    pros::delay(100);
    outpist.set_value(false);
    Stage_2.move_velocity(300);
    Stage_3.move_velocity(-600);
    chassis.turnToHeading(272,900);
    chassis.waitUntilDone();  
    chassis.setPose(chassis.getPose().x,leftdist.get_distance()/25.4,chassis.getPose().theta);

    outpist.set_value(false);
    chassis.waitUntilDone(); 
    intake.move_velocity(400);
    Stage_2.move_velocity(-600);
    Stage_3.move_velocity(600);
    int t_y=leftdist.get_distance()/25.4;         
    chassis.moveToPoint(16.3, t_y+2, 900, {.maxSpeed = 127, .minSpeed = 70});

    pros::delay(30); 
    load_1.set_value(true); 
    chassis.waitUntilDone();
    chassis.turnToPoint(5.6,48.9, 900,{.maxSpeed = 90, .minSpeed = 50});
      
    chassis.waitUntilDone();
        load_1.set_value(false); 
    chassis.moveToPoint(5.6,48.5, 900,{.maxSpeed = 90, .minSpeed = 50});
    chassis.waitUntilDone();chassis.waitUntilDone();

    outaking(1200); 
    //Step 4 : descore
    doinker.set_value(false);
     chassis.moveToPoint(34.9+7,19, 2000,{.forwards=false,.maxSpeed = 120, .minSpeed = 100});
    chassis.swingToHeading(180, DriveSide::RIGHT , 2000);
     chassis.moveToPoint(46.6+0.7,52, 2000,{.forwards=false,.maxSpeed = 120, .minSpeed = 50});

}



/*
++--------------------------------------------++
++--------------------------------------------++
|| _   _ _       _                         _  ||
||| | | (_) __ _| |__     __ _  ___   __ _| | ||
||| |_| | |/ _` | '_ \   / _` |/ _ \ / _` | | ||
|||  _  | | (_| | | | | | (_| | (_) | (_| | | ||
|||_| |_|_|\__, |_| |_|  \__, |\___/ \__,_|_| ||
||         |___/         |___/                ||
++--------------------------------------------++
++--------------------------------------------++

*/

void High_goal_1(){
  chassis.setPose(0, 0, 0);
  
  chassis.turnToHeading(345, 300);
  chassis.moveToPoint(-5,19, 1000,{.maxSpeed = 127, .minSpeed = 80,.earlyExitRange=2});
  pros::delay(30);
   load_1.set_value(true);
   chassis.moveToPoint(-10,32, 2000,{.maxSpeed = 127, .minSpeed = 80});
    pros::delay(30);
   load_1.set_value(false);
   chassis.swingToHeading(270,DriveSide::LEFT, 1000);
 
}
void High_goal() { // RED
  // should work middle goal broke

    //=============set pose ============================================================
    //step 1 : score aim match loader
  toss_color= "BLUE";
  chassis.setPose(0, 0, 270);
  chassis.moveToPoint(-25.3, 0, 800, {.maxSpeed = 127, .minSpeed = 110,.decelStartDist=4,.decelFactor=1.5,.useDistSensor=true,.distanceSenseTarget=13.3});
  load_1.set_value(true);
  chassis.turnToHeading(180, 900,{.maxSpeed = 127, .minSpeed = 100,});
  chassis.waitUntilDone();
  intake.move_velocity(600);
  Stage_2.move_velocity(-600);
  Stage_3.move_velocity(600);
  chassis.setPose(rightdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
    
    chassis.moveToPoint(20, -14.5, 1000, {.maxSpeed = 100, .minSpeed = 40,.useDistSensor=true,.distanceSenseTarget=3.5});
    pros::delay(200);
    chassis.moveToPoint(19,40, 1200, {.forwards = false, .maxSpeed = 127, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=31.5});
    long_goal(1400);
    load_1.set_value(false);
       
    //=============set pose ============================================================
    //step 2 : score aim high goal
    chassis.setPose(-36,24,180);
    pros::delay(100);
    outpist.set_value(false);
   intake.move_velocity(600);
  Stage_2.move_velocity(-600);
  Stage_3.move_velocity(600);
    chassis.turnToHeading(82,900);
    chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,rightdist.get_distance()/25.4,chassis.getPose().theta);
    outpist.set_value(false);
    chassis.waitUntilDone();
    intake.move_velocity(600);
    Stage_2.move_velocity(-600);
    Stage_3.move_velocity(600);
    int t_y=rightdist.get_distance()/25.4;
    chassis.moveToPoint(-21, t_y+1.5, 900, {.maxSpeed = 127, .minSpeed = 70});
    pros::delay(30); 
    load_1.set_value(true);
    
    
    chassis.waitUntilDone();
      toss_color="NO";
    chassis.turnToHeading(217-1, 1000,{.maxSpeed = 100, .minSpeed = 50,});
    chassis.moveToPoint(4.0,62, 900,{.forwards=false,.maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();chassis.waitUntilDone();
 
    middle_goal(400,1100);    


    load_1.set_value(false); 
    doinker.set_value(false);
    chassis.moveToPoint(-17+1.4-0.8,36, 2000,{.maxSpeed = 127, .minSpeed = 90,.earlyExitRange=2});
    chassis.turnToHeading(180, 2000,{.maxSpeed = 100, .minSpeed = 100,});
    chassis.waitUntilDone();
      toss_color= "BLUE";
    chassis.moveToPoint(-26+6.5-0.8, 58, 2000,{.forwards=false,.maxSpeed = 127, .minSpeed = 70,.useDistSensor=true,.distanceSenseTarget=52.5});

}

/*
++----------------------------------------------++
++----------------------------------------------++
|| ____        _            ___        ______   ||
||/ ___|  ___ | | ___      / \ \      / /  _ \  ||
||\___ \ / _ \| |/ _ \    / _ \ \ /\ / /| |_) | ||
|| ___) | (_) | | (_) |  / ___ \ V  V / |  __/  ||
|||____/ \___/|_|\___/  /_/   \_\_/\_/  |_|     ||
||                                              ||
++----------------------------------------------++
++----------------------------------------------++
*/
void soloAWP() { // RED
  

  // should work middle goal broke

    //=============set pose ============================================================
    //step 1: aim match loader

      //step 1 : score aim match loader
 // toss_color= "RED";
 toss_color="BLUE";
  chassis.setPose(0, 0, 270);
  chassis.moveToPoint(-25.3, 0, 800, {.maxSpeed = 127, .minSpeed = 110,.decelStartDist=4,.decelFactor=1.5,.useDistSensor=true,.distanceSenseTarget=13.3});
  load_1.set_value(true);
  chassis.turnToHeading(180, 900,{.maxSpeed = 127, .minSpeed = 100,});
  chassis.waitUntilDone();
  intake.move_velocity(400);
  Stage_2.move_velocity(-600);
  Stage_3.move_velocity(600);
  chassis.setPose(rightdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
    
    chassis.moveToPoint(20, -18.5, 1000, {.maxSpeed = 70, .minSpeed = 40,.useDistSensor=true,.distanceSenseTarget=2.5});
    pros::delay(200);
    chassis.moveToPoint(19,40, 1200, {.forwards = false, .maxSpeed = 127, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=31.5});
    long_goal(1400);
    load_1.set_value(false);
       
    //=============set pose ============================================================
    //step 2 : score aim high goal
    
    chassis.setPose(-36,24,180);
    pros::delay(30);
   
    Stage_2.move_velocity(300);
    Stage_3.move_velocity(-600);
    chassis.turnToHeading(82,900);
    chassis.waitUntilDone();
   
    chassis.setPose(chassis.getPose().x,rightdist.get_distance()/25.4,chassis.getPose().theta);
    outpist.set_value(false);
    chassis.waitUntilDone();
    intake.move_velocity(600);
    Stage_2.move_velocity(-600);
    Stage_3.move_velocity(600);
    int t_y=rightdist.get_distance()/25.4;
    chassis.moveToPoint(-21, t_y+1.5, 900, {.maxSpeed = 127, .minSpeed = 70});
    pros::delay(30); 
    load_1.set_value(true);
    
     toss_color="NO";
    chassis.waitUntilDone();
    chassis.turnToHeading(217-1, 1000,{.maxSpeed = 100, .minSpeed = 50,});
    chassis.moveToPoint(4.0,62, 900,{.forwards=false,.maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();chassis.waitUntilDone();
 
    middle_goal(600,1100);    



   load_1.set_value(false);
    //step 4: aim 2nd long goal
    chassis.moveToPoint(1,63, 900,{.maxSpeed = 127, .minSpeed = 90,.earlyExitRange=2});
    chassis.moveToPoint(61-6+1.5+0.9+4, 37, 2000, {.maxSpeed = 127, .minSpeed = 50,.useDistSensor=1,.distanceSenseTarget=11.9});
   toss_color="BLUE";
    chassis.turnToHeading(180, 900,{.maxSpeed = 127, .minSpeed = 100,});
         load_1.set_value(true);
    chassis.waitUntilDone();
    intake.move_velocity(600);
    Stage_2.move_velocity(-600);
    Stage_3.move_velocity(600);
    chassis.waitUntilDone();
    //=============set pose ============================================================
    chassis.setPose(leftdist.get_distance()/25.4,chassis.getPose().y,chassis.getPose().theta);
    pros::delay(20);
    chassis.moveToPoint(20-4.6-0.5, -10.5, 1000, {.maxSpeed = 70, .minSpeed = 40,.useDistSensor=true,.distanceSenseTarget=2.5});
    pros::delay(500);
    chassis.moveToPoint(19-2-0.8,42, 1200, {.forwards = false, .maxSpeed = 127, .minSpeed = 90,});
    long_goal(2000);
   


}


/*
++----------------------------------------------------++
++----------------------------------------------------++
||    _         _                ____  _    _ _ _     ||
||   / \  _   _| |_ ___  _ __   / ___|| | _(_) | |___ ||
||  / _ \| | | | __/ _ \| '_ \  \___ \| |/ / | | / __|||
|| / ___ \ |_| | || (_) | | | |  ___) |   <| | | \__ \||
||/_/   \_\__,_|\__\___/|_| |_| |____/|_|\_\_|_|_|___/||
||                                                    ||
++----------------------------------------------------++
++----------------------------------------------------++
*/

void Auton_Skil224ls() {
  
  toss_color ="NO";
  doinker.set_value(true);



  //=============set pose ====================================================
  //Step 1 : aim match loader
  chassis.setPose(0, 0, 270);
  intake.move_velocity(600);
  Stage_2.move_velocity(-600);
  Stage_3.move_velocity(600);
  pros::delay(20);
  load_1.set_value(true);
  chassis.moveToPoint(-26.9+1.5, 0, 900, {.maxSpeed = 127, .minSpeed = 70,.decelStartDist=4,.decelFactor=1.5,.useDistSensor=true,.distanceSenseTarget=9.5+1.5});
  chassis.turnToHeading(180, 900);
  chassis.moveToPoint(-36.4+1.5, -11.5, 800, {.maxSpeed = 127, .minSpeed = 40,.useDistSensor=true,});

  chassis.waitUntilDone();

  //====================================set pose =============================
  //Step 2 : go to the other side
  chassis.setPose(rightdist.get_distance()/25.4,5,chassis.getPose().theta);
  pros::delay(1200);
  chassis.moveToPoint(22, 10.00, 2000,{.forwards=false,.maxSpeed=120,.minSpeed=100,});
  chassis.turnToHeading(312-180,  621);
  load_1.set_value(false); 
  chassis.moveToPoint(5.9, 34.00, 2000,{.forwards=false,.maxSpeed=120,.minSpeed=100});
  chassis.turnToHeading(180, 900);
  chassis.moveToPoint(4.8,95+2, 2000,{.forwards=false,.maxSpeed=120,.minSpeed=100});
  chassis.waitUntilDone();
  chassis.turnToHeading(270, 900);
  chassis.waitUntilDone();

   //====================================set pose =============================
  //Step 3 :check distance and aim to goal
  chassis.setPose(frontdist.get_distance()/25.4,chassis.getPose().y,chassis.getPose().theta);
  pros::delay(40);
  chassis.moveToPoint(10.9,99.5+2, 2000,{.forwards=false,.maxSpeed=120,.minSpeed=110,.useDistSensor=true,.distanceSenseTarget=10.9});
  chassis.turnToHeading(0, 900);
  
  chassis.setPose(12,90.5+2,0);
  chassis.moveToPoint(12.5,90.5+2, 900,{.forwards=false,.maxSpeed=120,.minSpeed=100,.useDistSensor=true,.distanceSenseTarget=30});
  load_1.set_value(true);
  long_goal(1700); // score
 //====================================set pose =================================
   intake.move_velocity(600);
  Stage_2.move_velocity(-600);
  Stage_3.move_velocity(600);
   //Step 4 : get match load and aim to goal 
  chassis.setPose(leftdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
  chassis.moveToPoint(18, 75, 1200, {.maxSpeed = 60, .minSpeed = 40,});
  outpist.set_value(false);
  intake.move_velocity(600);
  Stage_2.move_velocity(-600);
  Stage_3.move_velocity(600);
  chassis.waitUntilDone();
  pros::delay(1300);
  chassis.moveToPoint(19, 20, 800, {.forwards=false,.maxSpeed = 127, .minSpeed = 100,.useDistSensor=true,.distanceSenseTarget=36});
  load_1.set_value(false);

  chassis.setPose(leftdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
 
  long_goal(1700); // score
  outpist.set_value(false);

   //=============set pose ============================================================
   //Step 5 : go over park zone and intake
chassis.turnToHeading(37, 500);
intake.move_velocity(600);
Stage_2.move_velocity(-600);
Stage_3.move_velocity(600);
chassis.moveToPoint(44, 64.8-0.9, 900, {.maxSpeed = 127, .minSpeed = 120,.earlyExitRange=4.3});
chassis.swingToHeading(85,DriveSide::RIGHT, 400);
  chassis.moveToPoint(69+5, 72, 3000, {.maxSpeed = 127, .minSpeed = 127,.useDistSensor=true,.distanceSenseTarget=27});
    chassis.turnToHeading(0, 900);
    chassis.waitUntilDone();

  //============== set pose===================================================
  //Step 5 : go over park zone and intake
      
  chassis.setPose(-rightdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
  pros:: delay(40);
  chassis.moveToPoint(-42, -30+4, 3000, {.forwards=false,.maxSpeed = 127, .minSpeed = 100,.useDistSensor=true,.distanceSenseTarget=30-4}); 
   chassis.turnToHeading(41, 500);
  chassis.moveToPoint(-59.8, -52.2, 1500, {.forwards=false,.maxSpeed = 127, .minSpeed = 30});  
   middle_goal(200,1000);
  chassis.moveToPoint(-20.5-1.5, -19, 3000, {.maxSpeed = 127, .minSpeed = 30});
  load_1.set_value(true);
    chassis.turnToHeading(0, 400);
    chassis.waitUntilDone();
  //============== set pose===================================================
  //Step 6 :aim 3rd long goal 
    chassis.setPose(rightdist.get_distance()/25.4,-frontdist.get_distance()/25.4,chassis.getPose().theta);
    int t_x=rightdist.get_distance()/25.4;
         
      chassis.moveToPoint(t_x+0.9, 7, 1300, {.maxSpeed = 127, .minSpeed =80});

     intake.move_velocity(600);
     Stage_2.move_velocity(-600);
     Stage_3.move_velocity(600);
    chassis.waitUntilDone();
    pros::delay(1200);
     chassis.moveToPoint(t_x+4, -37, 1000, {.forwards=false,.maxSpeed = 127, .minSpeed = 50,.useDistSensor=true,.distanceSenseTarget=1});

//====================================set pose==================================
   
  //Step 6 :go 4th long goal 
      chassis.setPose(rightdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
  t_x=rightdist.get_distance()/25.4;
      int t_y=frontdist.get_distance()/25.4;
      
   
     chassis.swingToHeading(180,DriveSide::LEFT  , 2000,{.direction= AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=127,.minSpeed=100});

  chassis.turnToHeading(180, 400); 
  chassis.moveToPoint(0,-29 ,2000,{.maxSpeed=120,.minSpeed=40,.useDistSensor=true,.distanceSenseTarget=22});
  chassis.waitUntilDone();
     
     chassis.turnToHeading(90, 900);

  chassis.waitUntilDone();
       chassis.setPose(-frontdist.get_distance()/25.4,chassis.getPose().y,chassis.getPose().theta);
     pros::delay(40);
  chassis.moveToPoint(-15.9,chassis.getPose().y ,2000,{.maxSpeed=120,.minSpeed=50,.useDistSensor=true,.distanceSenseTarget=19.5+2.5+1.9+1+1+1.5});
  
    chassis.turnToHeading(180, 900);

     chassis.waitUntilDone();
       
        //====================================set pose==================================

    //Step 7 :go aum long goal 
     
    t_x=leftdist.get_distance()/25.4;
    t_y=frontdist.get_distance()/25.4;
    pros::delay(40);
            load_1.set_value(true);
        chassis.moveToPoint(-13,-23 ,2000,{.forwards=false,.maxSpeed=120,.minSpeed=90});

        outpist.set_value(false);
        intake.move_velocity(600);
        Stage_2.move_velocity(-600);
        Stage_3.move_velocity(600);
   
    long_goal(1100);
    
    intake.move_velocity(600);
    Stage_2.move_velocity(-600);
    Stage_3.move_velocity(600);
    chassis.moveToPoint(-14.8,-50,2000,{.maxSpeed=120,.minSpeed=60,.useDistSensor=true,.distanceSenseTarget=2});
    outpist.set_value(false);
    pros::delay(1200);

    chassis.moveToPoint(-15,-20 ,2000,{.forwards=false,.maxSpeed=120,.minSpeed=90});
       
    long_goal(1100);
        load_1.set_value(false);
       //=============set pose ============================================================
           
       ///Step 8 :gopark/

    chassis.setPose(leftdist.get_distance()/25.4,frontdist.get_distance()/25.4,0);
    chassis.turnToHeading(37, 500);
    intake.move_velocity(600);
    Stage_2.move_velocity(-200);
    Stage_3.move_velocity(200);
   chassis.moveToPoint(42, 61, 900, {.maxSpeed = 127, .minSpeed = 120,.earlyExitRange=4.3});
   chassis.swingToHeading(86,DriveSide::RIGHT, 400);
   chassis.waitUntilDone();
    chassis.moveToPoint(44+14, 69, 3000, {.maxSpeed = 127, .minSpeed = 100});
  
   pros::delay(100);


}


void Auton_Skills(){
timedriving(200, true, 200);
}
