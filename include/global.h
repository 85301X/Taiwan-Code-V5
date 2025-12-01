#ifndef Global_h
#define Global_h


#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/gps.hpp"
#include "pros/motors.hpp"
#include "main.h"   
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
float apply_deadzone(float value, int factor);
extern void pidtesting();
extern pros::Controller controller;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Motor outake;
extern pros::Motor outake_2;
extern pros:: Motor hook;
extern pros::Motor intake;
extern pros:: Rotation odomy;
extern pros:: Motor hooks;
extern pros:: Optical optical;

extern pros::Imu inertial;
extern pros::Imu drivetrain_inertial;

extern pros::adi :: DigitalOut outpist;
extern pros::adi ::DigitalOut load_1;
extern pros::adi ::DigitalOut doinker;
extern pros::Rotation rotate;

//extern toss color
extern std::string toss_color;
extern int speed_switch;
extern bool hooks_spinning;
extern bool ejection;
extern lemlib::Drivetrain drivetrain;
extern lemlib::ControllerSettings linearController;
extern lemlib::ControllerSettings angularController;
extern lemlib::PID linearPID;
extern lemlib::PID angularPID;
extern bool intakeon;
extern lemlib:: Chassis chassis;
extern void print_task_fn(void* param);
extern void ring_toss_task_fn(void* param);
extern void auto_intake_jam();
extern void Auton_Skills();
extern void Auton_Skills_V2();
extern void Auton_Skills_V3();
extern void Auton_Skills_V4();
extern void intakemove();
extern void Blue_Right_AWP();
extern void Blue_Left_AWP();
extern void Blue_New_Left_AWP();
extern void goof_left_soloAWP();
extern void Blue_Right_New_AWP();
extern void austinRightAWP();
extern void updateLinearControllerSlew(float slew);
extern void simpleIntakeControl();
extern void auto_intake_jam();
extern void moveToWithHeading( double endHeading, double& targetX, double& targetY);
//extern pros:: Rotation odomy;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern pros:: Rotation odomy;
extern pros::Rotation odomx;
extern void soloAWP();
extern  pros:: GPS gps1;

#endif