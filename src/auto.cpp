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
#include <iostream>

void distancesetpose( pros::Distance distancesensor ){
  float df = frontdist.get_distance() / 25.4; // inches
  float dl = distancesensor.get_distance() / 25.4;  // inches
  float theta = chassis.getPose().theta;      // radians

  // Convert robot-frame distances into field coordinates
  float x = -dl * cos(theta) - df * sin(theta);
  float y = -dl * sin(theta) + df * cos(theta);

  chassis.setPose(x, y, theta);
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
    intake.move_velocity(-200);
    Stage_3.move_velocity(-600);
    Stage_2.move_velocity(600);

    pros::delay(400);

    //middle goal
    Stage_3.move_velocity(-velocity);
    Stage_2.move_velocity(-velocity);
    intake.move_velocity(600);
    pros::delay(time);
    Stage_2.move_velocity(0);
    pros::delay(100);

    }


void pidtesting(){
    chassis.setPose(0,0,90);
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
void Low_Goal() { // RED
  
  
    /*
    //=============set pose ============================================================
    //Step 1 :: Aim the loader
    chassis.setPose(0, 0, 90);
    
    chassis.moveToPoint(25.4+0.9, 0, 800, {.maxSpeed = 127, .minSpeed = 100,.decelStartDist=4,.decelFactor=1.5,.useDistSensor=true,.distanceSenseTarget=13.5-0.9});
    load_1.set_value(true);
    chassis.turnToHeading(180, 420,{.maxSpeed = 127, .minSpeed = 50,});
    chassis.waitUntilDone();
    intake.move_velocity(600);
    Stage_2.move_velocity(-600);
    Stage_3.move_velocity(600);
    chassis.waitUntilDone();

    //=============set pose ============================================================
    //Step 2 ::Match load and aim to goal

    chassis.setPose(leftdist.get_distance()/25.4,chassis.getPose().y,chassis.getPose().theta);
    chassis.moveToPoint(20-1.2, -6.5, 1000, {.maxSpeed = 127, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=3.5});
    pros::delay(600);
    chassis.moveToPoint(18.5-1.2,40, 1200, {.forwards = false, .maxSpeed = 127, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=31.5});
    long_goal(2000);
    load_1.set_value(false);
       
    
*/
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
    intake.move_velocity(600);
    Stage_2.move_velocity(-300);

    int t_y=leftdist.get_distance()/25.4;         
    chassis.moveToPoint(16.3, t_y+2, 900, {.maxSpeed = 127, .minSpeed = 70});
    pros::delay(40); 
    load_1.set_value(true); 
    chassis.waitUntilDone();
    chassis.turnToPoint(3,48, 900,{.maxSpeed = 90, .minSpeed = 50});
    chassis.waitUntilDone();
    chassis.moveToPoint(3,48, 900,{.maxSpeed = 90, .minSpeed = 50});
    chassis.waitUntilDone();chassis.waitUntilDone();
    pros::delay( 600);
    outaking(1200); 
    //Step 4 : descore
     chassis.moveToPoint(36.8,19, 2000,{.forwards=false,.maxSpeed = 120, .minSpeed = 50});
    chassis.swingToHeading(180, DriveSide::RIGHT , 2000);
     chassis.moveToPoint(49.,51, 2000,{.forwards=false,.maxSpeed = 120, .minSpeed = 50});
      
              
    
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
void Blue_Left_AWP(){
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
void Blue_Left_AWP_middle_goal_first() { // RED
  // should work middle goal broke

    //=============set pose ============================================================
  toss_color ="RED";
    chassis.setPose(0, 0, 270);
    
    chassis.moveToPoint(-25.4, 0, 800, {.maxSpeed = 127, .minSpeed = 100,.decelStartDist=4,.decelFactor=1.5,.useDistSensor=true,.distanceSenseTarget=13.5});
    load_1.set_value(true);
     chassis.turnToHeading(180, 900,{.maxSpeed = 127, .minSpeed = 100,});
         chassis.waitUntilDone();
           intake.move_velocity(600);
     Stage_2.move_velocity(-600);
     Stage_3.move_velocity(600);
    chassis.waitUntilDone();
    chassis.setPose(rightdist.get_distance()/25.4,chassis.getPose().y,chassis.getPose().theta);
      chassis.moveToPoint(20, -14.5, 1000, {.maxSpeed = 127, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=3.5});
   
     pros::delay(600);

  chassis.moveToPoint(19,40, 1200, {.forwards = false, .maxSpeed = 127, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=31.5});
      

    
    //outpist.set_value(false);
    long_goal(2000);
    
    load_1.set_value(false);
       

    



    chassis.setPose(-36,24,180);

    pros::delay(100);
      outpist.set_value(false);
         
Stage_2.move_velocity(300);
Stage_3.move_velocity(-600);
    chassis.turnToHeading(82,900);
        chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,rightdist.get_distance()/25.4,chassis.getPose().theta);
       outpist.set_value(false);
       
    chassis.waitUntilDone();
       
     intake.move_velocity(600);
    Stage_2.move_velocity(-300);

  int t_y=rightdist.get_distance()/25.4;
  chassis.moveToPoint(-23, t_y+2, 900, {.maxSpeed = 127, .minSpeed = 70});
pros::delay(40); 
   load_1.set_value(true); 
     
     chassis.waitUntilDone();
   
   
    chassis.turnToHeading(217-1, 1000,{.maxSpeed = 100, .minSpeed = 50,});
    
     chassis.waitUntilDone();
   
     
       chassis.moveToPoint(1,63, 900,{.forwards=false,.maxSpeed = 127, .minSpeed = 50});

  chassis.waitUntilDone();chassis.waitUntilDone();
         
       pros::delay( 1200);
       middle_goal(200,1100);
    
          load_1.set_value(false); 
          doinker.set_value(false);
              chassis.moveToPoint(-18.5 ,36, 2000,{.maxSpeed = 127, .minSpeed = 90,.earlyExitRange=2,.useDistSensor=true,.distanceSenseTarget=48.5});
   chassis.turnToHeading(180, 2000,{.maxSpeed = 100, .minSpeed = 100,});
    
   
     chassis.waitUntilDone();
        chassis.moveToPoint(-26+1.5,55, 2000,{.forwards=false,.maxSpeed = 127, .minSpeed = 40,.useDistSensor=true,.distanceSenseTarget=52.5});
     /*
      chassis.setPose(rightdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
    pros::delay(20);
              chassis.moveToPoint(25 ,52, 2000,{.forwards=false,.maxSpeed = 120, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=49});
              */
  
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
  
 /*
  // should work middle goal broke

    //=============set pose ============================================================
  toss_color ="RED";
    chassis.setPose(0, 0, 270);
    
    chassis.moveToPoint(-25.4, 0, 800, {.maxSpeed = 127, .minSpeed = 100,.decelStartDist=4,.decelFactor=1.5,.useDistSensor=true,.distanceSenseTarget=13.5});
    load_1.set_value(true);
     chassis.turnToHeading(180, 900,{.maxSpeed = 127, .minSpeed = 100,});
         chassis.waitUntilDone();
           intake.move_velocity(600);
     Stage_2.move_velocity(-600);
     Stage_3.move_velocity(600);
    chassis.waitUntilDone();
    chassis.setPose(rightdist.get_distance()/25.4,chassis.getPose().y,chassis.getPose().theta);
      chassis.moveToPoint(20, -14.5, 1000, {.maxSpeed = 127, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=3.5});
   
     pros::delay(600);

  chassis.moveToPoint(19,40, 1200, {.forwards = false, .maxSpeed = 127, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=31.5});
      

    
    //outpist.set_value(false);
    long_goal(2000);
    
    load_1.set_value(false);
       

    


*/
    chassis.setPose(-36,24,180);

    pros::delay(100);
      outpist.set_value(false);
         
Stage_2.move_velocity(300);
Stage_3.move_velocity(-600);
    chassis.turnToHeading(82,900);
        chassis.waitUntilDone();
    chassis.setPose(chassis.getPose().x,rightdist.get_distance()/25.4,chassis.getPose().theta);
pros::delay(40);
       outpist.set_value(false);
       
   
     intake.move_velocity(600);
    Stage_2.move_velocity(-300);

  int t_y=rightdist.get_distance()/25.4;
  chassis.moveToPoint(-23, t_y+2, 900, {.maxSpeed = 127, .minSpeed = 70});
pros::delay(40); 
   load_1.set_value(true); 
     

   
    chassis.turnToHeading(217-1, 1000,{.maxSpeed = 100, .minSpeed = 50,});
    
     
   
     
       chassis.moveToPoint(1,63, 900,{.forwards=false,.maxSpeed = 127, .minSpeed = 50});

  chassis.waitUntilDone();chassis.waitUntilDone();
         
       pros::delay( 1200);
       middle_goal(200,1100);
          chassis.moveToPoint(1,63, 900,{.maxSpeed = 127, .minSpeed = 90});
           chassis.moveToPoint(61-6, 37, 2000, {.maxSpeed = 127, .minSpeed = 120,.useDistSensor=1,.distanceSenseTarget=30});

    
     load_1.set_value(true);
     chassis.turnToHeading(180, 900,{.maxSpeed = 127, .minSpeed = 100,});
         chassis.waitUntilDone();
           intake.move_velocity(600);
     Stage_2.move_velocity(-600);
     Stage_3.move_velocity(600);
    chassis.waitUntilDone();
    chassis.setPose(leftdist.get_distance()/25.4,chassis.getPose().y,chassis.getPose().theta);
      chassis.moveToPoint(20, -14.5, 1000, {.maxSpeed = 127, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=3.5});
   
     pros::delay(600);

  chassis.moveToPoint(19,40, 1200, {.forwards = false, .maxSpeed = 127, .minSpeed = 60,.useDistSensor=true,.distanceSenseTarget=31.5});
      

    
    //outpist.set_value(false);
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

void Auton_Skills_v5() {
  
toss_color ="NO";
   doinker.set_value(true);
      //=============set pose ============================================================
    chassis.setPose(0, 0, 270);
        load_1.set_value(true);
    chassis.moveToPoint(-26.9, 0, 900, {.maxSpeed = 127, .minSpeed = 70,.decelStartDist=4,.decelFactor=1.5,.useDistSensor=true,.distanceSenseTarget=9.5});

     chassis.turnToHeading(180, 900);

    chassis.moveToPoint(-35.4, -11.5, 800, {.maxSpeed = 127, .minSpeed = 40,.useDistSensor=true,.distanceSenseTarget=5.5});
   
     intake.move_velocity(600);
     Stage_2.move_velocity(-600);
     Stage_3.move_velocity(600);
    chassis.waitUntilDone();
    //====================================set pose ============================================================v
    chassis.setPose(rightdist.get_distance()/25.4,5,chassis.getPose().theta);
     pros::delay(1400);



    chassis.moveToPoint(22, 10.00, 2000,{.forwards=false,.maxSpeed=120,.minSpeed=100,});
    chassis.turnToHeading(312-180,  621);
     load_1.set_value(false);
     
    
     chassis.moveToPoint(5.5, 34.00, 2000,{.forwards=false,.maxSpeed=120,.minSpeed=100});
  
//chassis.moveToPoint(-9, -9, 40000,{.forwards=false,.maxSpeed=120,.minSpeed=100});
     chassis.turnToHeading(180, 900);
  chassis.moveToPoint(4.8,95+2, 2000,{.forwards=false,.maxSpeed=120,.minSpeed=100,.useDistSensor=true,.distanceSenseTarget=30-2});
  chassis.waitUntilDone();

   chassis.turnToHeading(270, 900);
   chassis.waitUntilDone();
      chassis.setPose(frontdist.get_distance()/25.4,chassis.getPose().y,chassis.getPose().theta);
 
  pros::delay(40);
   chassis.moveToPoint(10.9,99.5+2, 2000,{.forwards=false,.maxSpeed=120,.minSpeed=110,.useDistSensor=true,.distanceSenseTarget=10.9});
     chassis.turnToHeading(0, 900);
   chassis.moveToPoint(12.5,90.5+2, 900,{.forwards=false,.maxSpeed=120,.minSpeed=100,.useDistSensor=true,.distanceSenseTarget=30});
      load_1.set_value(true);
    long_goal(1700);
      //====================================set pose ============================================================v
    chassis.setPose(leftdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
 
     chassis.moveToPoint(18, 72, 800, {.maxSpeed = 127, .minSpeed = 40,.useDistSensor=true,.distanceSenseTarget=0});
     outpist.set_value(false);
     intake.move_velocity(600);
     Stage_2.move_velocity(-600);
     Stage_3.move_velocity(600);
    chassis.waitUntilDone();
    pros::delay(1300);
     chassis.moveToPoint(19, 32, 800, {.forwards=false,.maxSpeed = 127, .minSpeed = 100,.useDistSensor=true,.distanceSenseTarget=36});
     load_1.set_value(false);


         long_goal(1700);
         
        
outpist.set_value(false);

   //=============set pose ============================================================
           chassis.setPose(leftdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
  chassis.turnToHeading(37, 500);
  intake.move_velocity(600);
  Stage_2.move_velocity(-200);
    Stage_3.move_velocity(200);
     chassis.moveToPoint(44, 65.8, 900, {.maxSpeed = 127, .minSpeed = 100,.earlyExitRange=3.8});
   chassis.swingToHeading(85,DriveSide::RIGHT, 400);

   chassis.moveToPoint(69, 69, 3000, {.maxSpeed = 127, .minSpeed = 100,.useDistSensor=true,.distanceSenseTarget=27});
  
       chassis.turnToHeading(0, 900);
       chassis.waitUntilDone();
       //============== set pose
      chassis.setPose(-rightdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
        chassis.moveToPoint(-42, -30+4, 3000, {.forwards=false,.maxSpeed = 127, .minSpeed = 100,.useDistSensor=true,.distanceSenseTarget=30-4});
      
   chassis.turnToHeading(41, 500);
       chassis.moveToPoint(-59, -53, 3000, {.forwards=false,.maxSpeed = 127, .minSpeed = 30});
         
      middle_goal(200,1000);

    
       chassis.moveToPoint(-20.5, -19, 3000, {.maxSpeed = 127, .minSpeed = 30});
        load_1.set_value(true);
       chassis.turnToHeading(0, 400);
       chassis.waitUntilDone();
         chassis.setPose(rightdist.get_distance()/25.4,-frontdist.get_distance()/25.4,chassis.getPose().theta);
        int t_x=rightdist.get_distance()/25.4;
         
      chassis.moveToPoint(t_x, 0, 1000, {.maxSpeed = 127, .minSpeed =50,.useDistSensor=true,.distanceSenseTarget=1});

     intake.move_velocity(600);
     Stage_2.move_velocity(-600);
     Stage_3.move_velocity(600);
    chassis.waitUntilDone();
    pros::delay(1200);
     chassis.moveToPoint(t_x, -37, 1000, {.forwards=false,.maxSpeed = 127, .minSpeed = 30,.useDistSensor=true,.distanceSenseTarget=1});

//====================================set pose==================================

      chassis.setPose(rightdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
     t_x=rightdist.get_distance()/25.4;
      int t_y=frontdist.get_distance()/25.4;
      
   
     chassis.swingToHeading(180,DriveSide::LEFT  , 2000,{.direction= AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed=127,.minSpeed=100});

  chassis.turnToHeading(180, 400); 
  chassis.moveToPoint(0,-25 ,2000,{.maxSpeed=120,.minSpeed=100,.useDistSensor=true,.distanceSenseTarget=25});
  chassis.waitUntilDone();
     
     chassis.turnToHeading(90, 900);

  chassis.waitUntilDone();
       chassis.setPose(frontdist.get_distance()/25.4,chassis.getPose().y,chassis.getPose().theta);
 
  chassis.moveToPoint(15.9,-37 ,2000,{.maxSpeed=120,.minSpeed=50,.useDistSensor=true,.distanceSenseTarget=13});
   chassis.waitUntilDone();
    chassis.turnToHeading(180, 900);

     chassis.waitUntilDone();
        //=============set pose ============================================================
      chassis.setPose(leftdist.get_distance()/25.4,frontdist.get_distance()/25.4,chassis.getPose().theta);
  
    t_x=leftdist.get_distance()/25.4;
    t_y=frontdist.get_distance()/25.4;
        chassis.moveToPoint(t_x ,t_y+15 ,2000,{.forwards=false,.maxSpeed=120,.minSpeed=100,.useDistSensor=true,.distanceSenseTarget=34});
    long_goal(1100);
   chassis.moveToPoint(t_x ,t_y-8 ,2000,{.maxSpeed=120,.minSpeed=60,.useDistSensor=true,.distanceSenseTarget=2});
  pros::delay(1200);
    chassis.moveToPoint(t_x ,t_y+15 ,2000,{.forwards=false,.maxSpeed=120,.minSpeed=100,.useDistSensor=true,.distanceSenseTarget=34});
    long_goal(1100);
       //=============set pose ============================================================
            chassis.setPose(leftdist.get_distance()/25.4,frontdist.get_distance()/25.4,0);
  chassis.turnToHeading(37, 500);
  intake.move_velocity(600);
  Stage_2.move_velocity(-200);
    Stage_3.move_velocity(200);
     chassis.moveToPoint(44, 65.8, 900, {.maxSpeed = 127, .minSpeed = 100,.earlyExitRange=3.8});
   chassis.swingToHeading(85,DriveSide::RIGHT, 400);

   chassis.moveToPoint(44+14, 69, 3000, {.maxSpeed = 127, .minSpeed = 100});
  
    
    
     
}
void Auton_Skills(){
    Auton_Skills_v5();


}




enum class Alliance {
    RED,
    BLUE
};

enum class AutonType {
    SOLO_AWP,
    HIGH_GOAL,
    LOW_GOAL
};

/*
Alliance selectedAlliance = Alliance::RED;
AutonType selectedAuton   = AutonType::SOLO_AWP;
bool autonConfirmed = false;



void drawAllianceScreen() {
    pros::screen::erase();

    switch (selectedAlliance) {
        case Alliance::RED:
           pros::screen::set_pen(pros::Color::red);
    pros::screen::fill_rect(5,5,480,480);
            break;
        case Alliance::BLUE:
              pros::screen::set_pen(pros::Color::blue);
      pros::screen::fill_rect(5,5,480,480);
            break;
    }

    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_LARGE, 120, 30, "SELECT ALLIANCE");

    pros::screen::print(
        pros::E_TEXT_LARGE,
        180,
        100,
        selectedAlliance == Alliance::RED ? "RED" : "BLUE"
    );

    pros::screen::print(pros::E_TEXT_SMALL, 120, 180, "LEFT: Toggle   A: Confirm");
}

void drawAutonScreen() {
    pros::screen::erase();

  
    switch (selectedAlliance) {
        case Alliance::RED:
           pros::screen::set_pen(pros::Color::red);
    pros::screen::fill_rect(5,5,480,480);
            break;
        case Alliance::BLUE:
              pros::screen::set_pen(pros::Color::blue);
      pros::screen::fill_rect(5,5,480,480);
            break;
    }

      pros::screen::set_pen(pros::Color::white);

    pros::screen::print(pros::E_TEXT_LARGE, 140, 30, "SELECT AUTON");

    const char* autonName;

    switch (selectedAuton) {
        case AutonType::SOLO_AWP:
            autonName = "SOLO AWP";
            break;
        case AutonType::HIGH_GOAL:
            autonName = "HIGH GOAL";
            break;
        case AutonType::LOW_GOAL:
            autonName = "LOW GOAL";
            break;
    }

    pros::screen::print(pros::E_TEXT_LARGE, 160, 100, autonName);
    pros::screen::print(pros::E_TEXT_SMALL, 120, 180, "LEFT: Cycle   A: Confirm");
}

void drawConfirmedScreen() {
    pros::screen::erase();
     switch (selectedAlliance) {
        case Alliance::RED:
           pros::screen::set_pen(pros::Color::red);
    pros::screen::fill_rect(5,5,480,480);
            break;
        case Alliance::BLUE:
              pros::screen::set_pen(pros::Color::blue);
      pros::screen::fill_rect(5,5,480,480);
            break;
    }

      pros::screen::set_pen(pros::Color::white);

    pros::screen::print(pros::E_TEXT_LARGE, 120, 80, "AUTON LOCKED");

    pros::screen::print(
        pros::E_TEXT_MEDIUM,
        140,
        130,
        selectedAlliance == Alliance::RED ? "RED" : "BLUE"
    );

    const char* autonName;

    switch (selectedAuton) {
        case AutonType::SOLO_AWP:  autonName = "SOLO AWP";  break;
        case AutonType::HIGH_GOAL: autonName = "HIGH GOAL"; break;
        case AutonType::LOW_GOAL:  autonName = "LOW GOAL";  break;
    }

    pros::screen::print(pros::E_TEXT_MEDIUM, 140, 160, autonName);
}



void autonSelector() {
    enum class MenuState {
        SELECT_ALLIANCE,
        SELECT_AUTON,
        CONFIRMED
    };

    MenuState state = MenuState::SELECT_ALLIANCE;

    drawAllianceScreen();

    while (!autonConfirmed && pros::competition::is_disabled()) {

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            switch (state) {

                case MenuState::SELECT_ALLIANCE:
                    selectedAlliance =
                        (selectedAlliance == Alliance::RED) ? Alliance::BLUE : Alliance::RED;
                    drawAllianceScreen();
                    break;

                case MenuState::SELECT_AUTON:
                    selectedAuton =
                        static_cast<AutonType>((static_cast<int>(selectedAuton) + 1) % 3);
                    drawAutonScreen();
                    break;

                default:
                    break;
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            switch (state) {

                case MenuState::SELECT_ALLIANCE:
                    state = MenuState::SELECT_AUTON;
                    drawAutonScreen();
                    break;

                case MenuState::SELECT_AUTON:
                    autonConfirmed = true;
                    state = MenuState::CONFIRMED;
                    drawConfirmedScreen();
                    break;

                default:
                    break;
            }
        }

        pros::delay(20);
    }
}
*/