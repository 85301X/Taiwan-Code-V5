#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "global.h"
#include "pros/adi.hpp"
#include "pros/gps.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
bool intakeon = false;
// motor groups
pros::MotorGroup leftMotors({-11 , -12,-13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({6, 7, 8}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros:: Gps gps1(21,-0.13,-0.095,-90);;

pros:: Motor intake(1 ,pros::MotorGearset::blue);
pros:: Motor Stage_3({2 },pros::MotorGearset::blue);
pros:: Motor Stage_2({3 },pros::MotorGearset::blue);


// Inertial Sensor on port 10
pros::Imu inertial(4); // 19 -> 8 because we're using new inertial sensor now


pros::Optical optical(10);


pros:: adi :: DigitalOut outpist('G');
pros:: adi :: DigitalOut load_1('H',false);
pros:: adi :: DigitalOut doinker('D');
 
// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed


// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation odomy(17 );



pros::Rotation odomx(16);
// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.5, // 10 inch track width
                              lemlib::Omniwheel:: NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(4.8, // proportional gain (kP)
                                              0.0, // integral gain (kI)
                                            20, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                              0// maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(5, // proportional gain (kP)
                                              0.7, // integral gain (kI)
                                              27.8, // derivative gain (kD)
                                              1.3, // anti windup
                                              2, // small error range, in inches
                                              40, // small error range timeout, in milliseconds
                                              2, // large error range, in inches
                                              40, // large error range timeout, in milliseconds
                                              100 // maximum acceleration (slew)
);
// horizontal tracking wheel encoder
// vertical tracking wheel encoder
// horizontal tracking wheel

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&odomy, lemlib::Omniwheel::NEW_2, 0);
lemlib::TrackingWheel horizontal_tracking_wheel(&odomx,lemlib::Omniwheel::NEW_2,-0.04); // -0.05 -> 0
// sensors for odometry
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &inertial // inertial sensor
);

lemlib:: ExpoDriveCurve throttle(
5,
10,
1.008

);

lemlib:: ExpoDriveCurve steer(
5,
10,
1.008

);
/*
lemlib:: ExpoDriveCurve throttle(
5,
60,
1.008

);

lemlib:: ExpoDriveCurve steer(
5,
30,
1.008

);
*/

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors,  &throttle,&steer);
