
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
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
#include <iostream>

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
	pros::lcd::print(0, "Runing");
    chassis.calibrate(); // calibrate sensors
    controller.clear();
    gps1.set_data_rate(5);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    odomy.set_reversed(true);
    optical.set_integration_time(5);
    odomx.set_reversed(true);
   
        }

/**
 * Runs while the robot is disabled
 */
void disabled() {

}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {

}


// get a path used for pure pursuit
// this needs to be put outside a function


/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    
  
   soloAWP();
}

/**
 * Runs in driver control
 */


float drive_speed_factor = 1.0;
float turn_speed_factor = 1.2;
bool directionmode = true;
double drive_left_speed=0;
double drive_right_speed=0;
bool Stage_3stopped = true;
bool clamped = true;
bool doinked = true;
bool hooking = false;
bool hang = true;
int speed_switch = 1;
bool intake_spinning = true;
bool ejection = false;
std:: string detected_color;
std:: string toss_color = "BLUE";
bool tossing =false;


void color_sort() {
    // Read optical sensor values
    int hue = optical.get_hue();
    int proximity = optical.get_proximity();

    // -----------------------------
    // 1. COLOR DETECTION
    // -----------------------------
    if (proximity > 100) {
        if (hue >= 100 && hue <= 200) {
            detected_color = "BLUE";
        }
        else if (hue >= 10 && hue <= 21) {
            detected_color = "RED";
        }
        else {
            detected_color = "NO";
        }
    }
    else {
        detected_color = "NO";
    }

    // -----------------------------
    // 2. SORTING DECISION
    // -----------------------------
    const bool should_toss = 
        (detected_color == toss_color) && 
        (toss_color != "NO") &&
        !tossing;                 // avoids double-triggering

    if (!should_toss) return;

    // -----------------------------
    // 3. PERFORM TOSSING ACTION
    // -----------------------------
    tossing = true;

    // Store current velocity to restore later
    double previous_velocity = Stage_3.get_actual_velocity();

    // Stop everything before reversing
    Stage_3.move_velocity(0);
    Stage_2.move_velocity(0);

    // Reverse to eject incorrect triball
    Stage_3.move_velocity(-600);
    Stage_2.move_velocity(-600);

    pros::delay(500);

    // -----------------------------
    // 4. RESTORE PREVIOUS STATE
    // -----------------------------
    if (previous_velocity > 100) {
        Stage_3.move_velocity(previous_velocity);
        Stage_2.move_velocity(previous_velocity);
    }
    else {
        Stage_3.move_velocity(0);
        Stage_2.move_velocity(0);
    }

    tossing = false;
}

void print_task_fn(void *param){
    //print imu inertial heading
   
    while (true){
       
     // color_sort();
        pros::lcd::print(1, "heading: %f", inertial.get_heading());
        pros::lcd::print(2, "x: %f", chassis.getPose().x);
        pros::lcd::print(3, "y: %f", chassis.getPose().y);
           pros::lcd::print(1, "heading: %f", inertial.get_heading());
      
        pros::lcd::print(4, "in: %f",Stage_3.get_actual_velocity());
     pros::lcd::print(5,  "Color: %s", toss_color);
        pros::lcd::print(6,  "Color: %s", detected_color);
        //print toss color
        
        //print on controller
        //print on controller
      //  controller.print(1, 0, "x: %.2f,y:%.2f,I:%.2f", chassis.getPose().x,chassis.getPose().y,inertial.get_heading());
     
        controller.print(1, 0, "in: %.0f ,y: %.0f,,x:%.0f c:%s",  inertial.get_heading(),chassis.getPose().y,chassis.getPose().x,toss_color);

        pros::delay(100);
}
}
bool loaded_2 = false;
bool loaded_1 = false;






void opcontrol() {
    controller.clear();


  
   // pros::Task antijam_task(antijam_task_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT);
    //use print task to print imu heading and position
    pros::Task print_task(print_task_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT);
    Stage_3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
         Stage_3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        int raw_forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        int raw_turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    

    // ----- Use your drive method with adjusted inputs -----
        if (directionmode){ 
              chassis.curvature(raw_forward, raw_turn);
            }
        else{
                 chassis.curvature(-raw_forward, -raw_turn);
            }
      

        //========================================================================================================
        //  color sort done
    // Button controls for intake and outtake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // score
              outpist.set_value(true);
            if (tossing == false ){
            Stage_3.move_velocity(600);
            Stage_2.move_velocity(-600);
            }
            intake.move_velocity(600);
            Stage_3stopped = false;
            intake_spinning = true;
               outpist.set_value(false);
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // score
              outpist.set_value(false);
            if (tossing == false ){
            Stage_3.move_velocity(600);
            Stage_2.move_velocity(-600);
            }
            intake.move_velocity(600);
            Stage_3stopped = false;
            intake_spinning = true;
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) { // reverse intake 
              outpist.set_value(false);
              if (tossing == false ){
            Stage_3.move_velocity(-600);
            Stage_2.move_velocity(600);
            }
            intake.move_velocity(-600);
            Stage_3stopped = false;
            intake_spinning = true;
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // cahnged
              outpist.set_value(false);
            intake.move_velocity(600);
             if (tossing == false ){
            Stage_3.move_velocity(-600);
              Stage_2.move_velocity(-600);
            }
            intake_spinning = true;
            Stage_3stopped = false;
        } 
        else {  // No buttons pressed
            if (!Stage_3stopped) {
                Stage_3.move_velocity(0);
                Stage_2.move_velocity(0);
                Stage_3stopped = true;
            }
            if (intake_spinning) {
                intake.move_velocity(0);
                intake_spinning = false;
            }
            outpist.set_value(false);
        }
        //========================================================================================================
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            //switch toss color from no > red > blue > no
            if (toss_color == "RED")
            {
                toss_color = "BLUE";
            }
            
            else if (toss_color == "BLUE")
            {
                toss_color = "RED";

        }
    }


         if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            
           autonomous();
             
        }
         if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            
            directionmode = !directionmode;
            
             
        }
        // === Pneumatic F toggle (L1) ===
     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
                load_1.set_value(loaded_1);
                loaded_1 = !loaded_1;  
        }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
                doinker.set_value(doinked);
                doinked = !doinked;  
        }
        //doink code done
       


     
  
        // delay to save resources
        pros::delay(25);
        
    }
}/**/