#ifndef Global_h
#define Global_h


#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/motors.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
extern pros::Controller controller;

extern lemlib:: Chassis chassis;

// motor groups
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::Motor Stage_3;
extern pros::Motor Stage_2;
extern pros:: Motor hook;
extern pros::Motor intake;
extern pros:: Motor hooks;
extern pros:: Optical optical;

//inertial
extern pros::Imu inertial;

//pistors
extern pros::adi :: DigitalOut outpist;
extern pros::adi ::DigitalOut load_1;
extern pros::adi ::DigitalOut doinker;

//distance sensor
extern pros::Distance frontdist;
extern pros::Distance leftdist;
extern pros::Distance rightdist;

//Varaibles
extern std::string toss_color;
extern bool hooks_spinning;
extern bool intakeon;
extern void print_task_fn(void* param);


//autonomous
extern void Auton_Skills();
extern void Low_Goal();
extern void Blue_Left_AWP();
extern void soloAWP();
extern void pidtesting();
extern void autonSelector();


//Other sensor;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern pros:: Rotation odomy;
extern pros::Rotation odomx;
extern  pros:: GPS gps1;

#endif