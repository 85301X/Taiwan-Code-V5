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
/*
void antijam_task_fn(void* param){
    while (true){
        simpleIntakeControl();
        pros::delay(10);
    }
}

void autoantijamtask(void* param){
    while (true){
        auto_intake_jam();
        pros::delay(10);
    }
}
	*/
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
	pros::lcd::print(0, "sigma skibidi balls");
    chassis.calibrate(); // calibrate sensors
    controller.clear();
     gps1.set_data_rate(5);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
   
    //pros clear screen
  

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
   
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
    // Testing_code();
  // pros:: Task an(autoantijamtask, (void*)"PROS", TASK_PRIORITY_DEFAULT);
    // Auton_Skills_V3();
    // Auton_Skills_V3();
 
//   Blue_Right_AWP();
	// soloAWP();
    pidtesting();
}

/**
 * Runs in driver control
 */

float apply_deadzone(float value, int factor){
    if (std:: abs(value) > 5){
        return (0.5 * std::abs(value / factor) * (value / factor)) ;
    }
    else{
        return 0.0;
    }
}

float drive_speed_factor = 1.0;
float turn_speed_factor = 1.2;
bool directionmode = true;
double drive_left_speed=0;
double drive_right_speed=0;
bool outakestopped = true;
bool clamped = true;
bool doinked = true;
bool hooking = false;
bool hang = true;
int speed_switch = 1;
bool intake_spinning = true;
bool ejection = false;


void print_task_fn(void *param){
    //print imu inertial heading
   
    while (true){
        pros::lcd::print(1, "heading: %f", inertial.get_heading());
        pros::lcd::print(2, "x: %f", chassis.getPose().x);
        pros::lcd::print(3, "y: %f", chassis.getPose().y);
           pros::lcd::print(1, "heading: %f", inertial.get_heading());
        pros::lcd::print(2, "gps_x: %f",gps1.get_position().x*39.37);

        pros::lcd::print(3, "gps_y: %f", gps1.get_position().y*39.37);
        pros::lcd::print(4, "in: %f",gps1.get_heading());
        //print on controller
        //print on controller
     //   controller.print(1, 0, "x: %.2f,y:%.2f,I:%.2f", chassis.getPose().x,chassis.getPose().y,inertial.get_heading());
        
     //   controller.print(4, 0, "x: %f",  gps1.get_position().x);
      //  controller.print(5, 0, "y: %f", gps1.get_position().y);
        controller.print(1, 0, "in: %f",  inertial.get_heading());

        pros::delay(100);
}
}
bool loaded_2 = false;
bool loaded_1 = false;


void opcontrol() {
    controller.clear();
    controller.clear_line(2);

  
   // pros::Task antijam_task(antijam_task_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT);
    //use print task to print imu heading and position
    pros::Task print_task(print_task_fn, (void*)"PROS", TASK_PRIORITY_DEFAULT);
    outake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
         outake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        int raw_forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        int raw_turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    

    // ----- Use your drive method with adjusted inputs -----
        if (directionmode){ 
                 chassis.curvature(raw_forward, raw_turn);
            }
        else{
                 chassis.curvature(-raw_forward, raw_turn);
            }
      

        //========================================================================================================
        //  color sort done
    // Button controls for intake and outtake
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            outake.move_velocity(600);
            intake.move_velocity(600);
            outakestopped = false;
            intake_spinning = true;
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            outake.move_velocity(-600);
            intake.move_velocity(-600);
            outakestopped = false;
            intake_spinning = true;
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move_velocity(600);
            outake.move_velocity(-600);
            intake_spinning = true;
            outakestopped = false;
        } 
        else {  // No buttons pressed
            if (!outakestopped) {
                outake.move_velocity(0);
                outakestopped = true;
            }
            if (intake_spinning) {
                intake.move_velocity(0);
                intake_spinning = false;
            }
        }

         if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            
           autonomous();
             
        }
         if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            
            directionmode = !directionmode;
            
             
        }
        // === Pneumatic F toggle (L1) ===
     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
                load_1.set_value(loaded_1);
                loaded_1 = !loaded_1;  
        }
        //doink code done
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){       
                load_2.set_value(loaded_2);
                loaded_2 = !loaded_2;    
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            linearController.kP += 0.1;
            
         controller.print(2, 0, "kP: %.3f,KD:%.3f", linearController.kP,linearController.kD);
         }
    
       if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
           linearController.kD += 0.1;
        //    linearController.kD = linearController.kD + 1;
                  controller.print(2, 0, "kP: %.3f,KD:%.3f", linearController.kP,linearController.kD);
        }

       if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
           chassis.calibrate();
           pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
       }

    //     if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
    //         linearController.kP += 1;
            
    //      controller.print(1, 0, "kP: %f", linearController.kP);
    //      }
    
    //    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
    //        linearController.kD += 1;
    //        controller.print(1, 0, "kD: %f", linearController.kD);
    //     }
       
        // delay to save resources
        pros::delay(25);
        
    }
}