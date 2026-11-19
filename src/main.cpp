#include "RclTracking.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/misc.h"
#include <csignal>
#include <cstdio>
#include "main.h"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "func_add.hpp"

using namespace pros;
using namespace lemlib;

RclTracking RclMain(&chassis);

void initialize() {
	lcd::initialize();

	// rotation reset
	lift_rotation.reset_position();
	lift.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	//chassis initilize
	chassis.calibrate();
	chassis.setPose(-60.9, 0, 270, false);
	RclMain.setRclPose(Pose(-60.9, 0, 270));
	chassis.setBrakeMode(E_MOTOR_BRAKE_HOLD);
	RclMain.startTracking();

	Task macro_toggle_task(macro_toggleIntake);

	Task([&] {
		while (true) {
			lcd::print(0, "X: %.1f", chassis.getPose().x);
            lcd::print(1, "Y: %.1f", chassis.getPose().y);
            lcd::print(2, "Theta: %.1f", fmod(chassis.getPose().theta, 360.0));
			delay(50);
    	}
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	Task auto_lift_task(lift_auto); // start macro tasks
	liftTarget = 17000;
	toggleRed = true; // set toggle to red
	claw.move(127);
	delay(750); // wait for toggle to turn red

	// preload -> goal #1
	chassis.moveToPoint(-47, 0, 800, {.forwards = false, .maxSpeed = 90});
	chassis.turnToPoint(-47.5, -20, 700, {.forwards = false, .maxSpeed = 70});
	chassis.moveToPose(-47.5, -20, 0, 1300, {.forwards = false});
	chassis.waitUntilDone();
	RclMain.updateBotPose(&left_rcl);
	liftTarget = -200;
	delay(500);
	claw.move(-127);
	delay(200);
	chassis.moveToPoint(-47, -6, 800);
	chassis.waitUntilDone();

	// stack #1 -> goal #1
	chassis.turnToPoint(-29, -21, 500, {.forwards = false});
	chassis.waitUntilDone();
	chassis.moveToPoint(-29, -21, 1800, {.forwards = false, .maxSpeed = 40});
	claw.move(-127);
	chassis.waitUntilDone();
	claw.move(127);
	delay(300);
	liftTarget = 32000;
	delay(200);
	chassis.moveToPose(-45, -26, 90, 2000, {.forwards = false});
	chassis.waitUntilDone();
	RclMain.setRclPose(Pose(-39.5, -23.5));
	RclMain.updateBotPose(&right_rcl);
	liftTarget = 21000;
	delay(500);
	claw.move(-127);
	delay(500);

	// pin -> toggle #2
	chassis.moveToPoint(-23.5, -23.5, 800, {.maxSpeed = 110});
	chassis.turnToHeading(180, 500);
	claw.move(0);
	chassis.moveToPoint(-23.5, -8, 2400, {.forwards = false, .maxSpeed = 30});
	liftTarget = 10000;
	claw.move(127);
	chassis.waitUntilDone();
	delay(1300);
	liftTarget = 21000;
	chassis.moveToPose(-5, -60, 180, 2500);
	chassis.waitUntilDone();
	delay(800);

	// goal #2
	chassis.moveToPoint(-5, -47, 800, {.forwards = false, .maxSpeed = 90});
	chassis.turnToHeading(90, 1000, {.maxSpeed = 70});
	chassis.moveToPose(-16, -47, 90, 800, {.forwards = false});
	chassis.waitUntilDone();
	RclMain.setRclPose(Pose(-16, -47));
	RclMain.updateBotPose(&right_rcl);
	liftTarget = -200;
	delay(500);
	claw.move(-127);
	delay(200);
	chassis.moveToPoint(-6, -47, 800);
	chassis.waitUntilDone();

	// stack -> goal #2
	chassis.moveToPose(-13, -58.5, 90, 1300, {.forwards = false, .lead = 0.1});
	chassis.moveToPoint(-34, -58.5, 800, {.forwards = false});
	chassis.turnToPoint(-42.5, -51.5, 500, {.forwards = false});
	chassis.moveToPoint(-42.5, -51.5, 1800, {.forwards = false, .maxSpeed = 40});
	claw.move(-127);
	chassis.waitUntilDone();
	claw.move(127);
	delay(500);
	liftTarget = 32000;
	delay(300);
	chassis.moveToPose(-31, -47, 270, 2500, {.forwards = false});
	chassis.waitUntilDone();
	RclMain.setRclPose(Pose(-31, -47));
	RclMain.updateBotPose(&front_rcl);
	RclMain.updateBotPose(&left_rcl);
	liftTarget = 21000;
	delay(500);
	claw.move(-127);
	delay(500);

	// matchload -> goal #1  FOUR TIMES
	chassis.moveToPoint(-38, -47, 500);
	chassis.turnToPoint(-59, -59, 500, {.forwards = false});
	claw.move(127);
	liftTarget = -200;
	chassis.moveToPose(-59, -59, 90, 2000, {.forwards = false, .lead = 0.2});
	chassis.waitUntilDone();
	RclMain.updateBotPose(&right_rcl);
	
}

void opcontrol() {
	Task macro_lift_task(macro_lift);
	manual = false;

	while (true) {
		// get left y and right x positions
        int leftY = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX, false, 0.5);

		// flipper control
		if (lift_rotation.get_position() < 20000 || level == 0) { /*if lift is lower than a certain height/level
																	then set to retracted position*/
			flipperState = false;
		} else {
			if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) { // else can be changed freely
				flipperState = !flipperState;
			}
		}
		flipper.set_value(flipperState);

		// claw control
		if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			claw.move(-127);
		} else {
			claw.move(127);
		}

		// manual toggle
		if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			manual = !manual;
		}

		// what to do if manual or not
		if (manual) {
			if (master.get_digital(E_CONTROLLER_DIGITAL_Y)) {
				lift.move(127);
			} else if (master.get_digital(E_CONTROLLER_DIGITAL_RIGHT)) {
				lift.move(-80);
			} else {
				lift.move(0);
				lift.brake();
			}
		} else {
			if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
				if (level < 4) {
					level++;
				}
			} else if (master.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
				if (level > 0) {
					level--;
				}
			}
		}

		delay(20);
	}
}