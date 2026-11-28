#include "RclTracking.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/distance.hpp"

// left motor group
inline pros::MotorGroup left_motor_group({-2, -3, -4}, pros::MotorGears::blue);
// right motor group
inline pros::MotorGroup right_motor_group({5, 6, 7}, pros::MotorGears::blue);

// drivetrain settings
inline lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                                     &right_motor_group, // right motor group
                                     10.79, // 10.79 inch track width
                                     lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                                     450, // drivetrain rpm is 450
                                     2 // horizontal drift is 2 (for now)
);

// imu
inline pros::Imu imu(11);

// vertical tracking wheel
inline pros::Rotation vertical_encoder(1);
inline lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.31);

// odometry settings
inline lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical 1
                                   nullptr, // no 2nd vertical
                                   nullptr, // no horizontal
                                   nullptr, // no horizontal
                                   &imu // inertial sensor
);

// lateral PID controller
inline lemlib::ControllerSettings lateral_controller(9, // proportional gain (kP)
                                                     0, // integral gain (kI)
                                                     50, // derivative gain (kD)
                                                     3, // anti windup
                                                     2, // small error range, in inches
                                                     40, // small error range timeout, in milliseconds
                                                     5, // large error range, in inches
                                                     100, // large error range timeout, in milliseconds
                                                     20 // maximum acceleration (slew)
);

// angular PID controller
inline lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                                     0, // integral gain (kI)
                                                     50, // derivative gain (kD)
                                                     3, // anti windup
                                                     1, // small error range, in degrees
                                                     100, // small error range timeout, in milliseconds
                                                     3, // large error range, in degrees
                                                     500, // large error range timeout, in milliseconds
                                                     0 // maximum acceleration (slew)
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, // drivetrain settings
                               lateral_controller, // lateral PID settings
                               angular_controller, // angular PID settings
                               sensors // odometry sensors
);

inline pros::Motor intake(12);
inline pros::MotorGroup lift({-8, 9});
inline pros::Motor claw(-20);

inline pros::Rotation lift_rotation(10);
inline pros::Optical optical(17);

inline pros::ADIDigitalOut flipper('A');

inline pros::Controller master(pros::E_CONTROLLER_MASTER);

// distance sensors
inline pros::Distance right_dist(14);
inline pros::Distance front_dist(15);
inline pros::Distance left_dist(13);

inline RclSensor front_rcl(&front_dist, 4.63, 7.28, 0, 8.0);
inline RclSensor right_rcl(&right_dist, 6.2, 5.71, 90, 8.0);
inline RclSensor left_rcl(&left_dist, -6.2, 5.71, 270, 8.0);

// obstacles
inline Circle_Obstacle center_goal(0, 0, 7.95);

inline Circle_Obstacle red_goal_left(-47, -23.5, 7.95);
inline Circle_Obstacle red_goal_right(-23.5, -47, 7.95);
inline Circle_Obstacle blue_goal_left(47, 23.5, 7.95);
inline Circle_Obstacle blue_goal_right(23.5, 47, 7.95);

inline Circle_Obstacle neutral_goal_redLeft(-47, 23.5, 7.95);
inline Circle_Obstacle neutral_goal_redRight(23.5, -47, 7.95);
inline Circle_Obstacle neutral_goal_blueLeft(47, -23.5, 7.95);
inline Circle_Obstacle neutral_goal_blueRight(-23.5, 47, 7.95);

inline Circle_Obstacle loader_redRight(-68.13, -58.8, 3.74);
inline Circle_Obstacle loader_redLeft(-68.13, 58.8, 3.74);
inline Circle_Obstacle loader_blueRight(68.13, 58.8, 3.74);
inline Circle_Obstacle loader_blueLeft(68.13, -58.8, 3.74);