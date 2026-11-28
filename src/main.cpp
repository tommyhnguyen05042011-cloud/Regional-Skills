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
	lift_rotation.set_position(0);
	lift.set_brake_mode(E_MOTOR_BRAKE_HOLD);

	//chassis initilize
	chassis.calibrate();
	chassis.setPose(-60.9, 0, 270, false);
	RclMain.setRclPose(Pose(-60.9, 0, 270));
	RclMain.startAccumulating();
	chassis.setBrakeMode(E_MOTOR_BRAKE_HOLD);
	RclMain.startTracking();

	Task macro_toggle_task(macro_toggleIntake);

	Task([&] {
		while (true) {
			lcd::print(0, "X: %.1f | Y: %.1f | Theta: %.1f",
					   chassis.getPose().x, chassis.getPose().y, fmod(chassis.getPose().theta, 360.0));
			lcd::print(1, "Lift Position: %d", lift_rotation.get_position());
			delay(100);
    	}
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	Task auto_lift_task(lift_auto); // start macro tasks
	liftTarget = 24000;
	toggleRed = true; // set toggle to red
	claw.move(127);
	delay(750); // wait for toggle to turn red
	RclMain.stopAccumulating();

	// preload -> goal #1
	chassis.moveToPoint(-50, 0, 700, {.forwards = false, .maxSpeed = 80});
	chassis.turnToPoint(-47, -20, 700, {.forwards = false, .maxSpeed = 70});
	RclMain.updateBotPose(&left_rcl);
	RclMain.updateBotPose(&front_rcl);
	chassis.moveToPoint(-47, -17, 1300, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	RclMain.updateBotPose(&left_rcl);
	RclMain.startAccumulating();
	delay(300);
	liftTarget = -5000;
	delay(400);
	claw.move(-127);
	delay(200);
	RclMain.stopAccumulating();
	chassis.cancelMotion();
	chassis.moveToPoint(-47, 0, 800, {.maxSpeed = 100});
	chassis.waitUntilDone();
	RclMain.updateBotPose(&left_rcl);

	// stack #1 -> goal #1
	chassis.turnToPoint(-26, -22.5, 400, {.forwards = false});
	claw.move(127);
	chassis.moveToPoint(-26, -22.5, 500, {.forwards = false, .maxSpeed = 70});
	chassis.moveToPoint(-26, -22.5, 2000, {.forwards = false, .maxSpeed = 30});
	RclMain.updateBotPose(&left_rcl);
	chassis.waitUntilDone();
	RclMain.updateBotPose(&front_rcl);
	RclMain.updateBotPose(&left_rcl);
	delay(400);
	liftTarget = 38000;
	chassis.moveToPoint(-21.5, -27, 500, {.forwards = false});
	chassis.turnToHeading(90, 1000, {.maxSpeed = 50});
	chassis.moveToPose(-42.5, -27, 90, 1500, {.forwards = false});
	chassis.waitUntilDone();
	liftTarget = 21000;
	delay(500);
	RclMain.setRclPose(Pose(-39.5, -23.5));
	RclMain.updateBotPose(&right_rcl);
	RclMain.startAccumulating();
	claw.move(-127);
	delay(500);
	RclMain.stopAccumulating();

	// stack #2 -> goal #1
	chassis.moveToPoint(-23.5, -23.5, 800);
	chassis.turnToPoint(-45, -44.5, 800, {.forwards = false});
	liftTarget = -5000;
	chassis.moveToPoint(-45, -44.5, 2800, {.forwards = false, .maxSpeed = 30});
	claw.move(127);
	chassis.waitUntilDone();
	RclMain.updateBotPose(&left_rcl);
	RclMain.updateBotPose(&right_rcl);
	delay(800);
	liftTarget = 70000;
	delay(500);
	chassis.moveToPoint(-49, -47.5, 800, {.forwards = false});
	chassis.turnToHeading(180, 1000, {.maxSpeed = 40});
	chassis.moveToPoint(-50, -25, 1200, {.forwards = false, .maxSpeed = 60});
	chassis.waitUntilDone();
	delay(300);
	liftTarget = 45000;
	delay(500);
	claw.move(-127);
	RclMain.setRclPose(Pose(-47, -31));
	RclMain.updateBotPose(&right_rcl);
	RclMain.updateBotPose(&front_rcl);
	RclMain.startAccumulating();
	delay(800);

	// matchload pin -> goal #2
	chassis.moveToPoint(-47, -58.5, 1000, {.maxSpeed = 70});
	delay(100);
	liftTarget = -2000;
	chassis.turnToHeading(90, 500, {.maxSpeed = 90});
	claw.move(127);
	chassis.moveToPoint(-62, -58.5, 2000, {.forwards = false, .maxSpeed = 40});
	chassis.moveToPoint(-47, -46, 1200, {.maxSpeed = 60});
	delay(300);
	liftTarget = 24000;
	chassis.turnToHeading(270, 1500, {.maxSpeed = 60});
	chassis.moveToPoint(-32, -46, 1300, {.forwards = false, .maxSpeed = 50});
	chassis.waitUntilDone();
	RclMain.setRclPose(Pose(-31, -47));
	RclMain.updateBotPose(&left_rcl);
	liftTarget = -2000;
	delay(500);
	claw.move(-127);
	delay(200);

	// matchload stack #1 -> goal #2
	chassis.moveToPoint(-47, -58.5, 1000, {.maxSpeed = 90});
	chassis.turnToHeading(90, 1000, {.maxSpeed = 90});
	claw.move(127);
	liftTarget = -2000;
	chassis.moveToPoint(-62, -58.5, 1500, {.forwards = false, .maxSpeed = 40});
	chassis.moveToPoint(-47, -47, 1200, {.maxSpeed = 50});
	delay(300);
	liftTarget = 38000;
	chassis.turnToHeading(270, 1500, {.maxSpeed = 60});
	chassis.moveToPoint(-31, -47, 1300, {.forwards = false, .maxSpeed = 70});
	chassis.waitUntilDone();
	RclMain.setRclPose(Pose(-31, -47));
	RclMain.updateBotPose(&left_rcl);
	liftTarget = 21000;
	delay(500);
	claw.move(-127);
	delay(200);

	// // matchload stack #2 -> goal #2
	// chassis.moveToPoint(-47, -58.5, 1000, {.maxSpeed = 90});
	// chassis.turnToHeading(90, 1000, {.maxSpeed = 90});
	// claw.move(127);
	// liftTarget = -2000;
	// chassis.moveToPoint(-62, -58.5, 1500, {.forwards = false, .maxSpeed = 40});
	// chassis.moveToPoint(-47, -47, 1200, {.maxSpeed = 50});
	// delay(300);
	// liftTarget = 70000;
	// chassis.turnToHeading(270, 1500, {.maxSpeed = 60});
	// chassis.moveToPoint(-31, -47, 1300, {.forwards = false, .maxSpeed = 70});
	// chassis.waitUntilDone();
	// RclMain.setRclPose(Pose(-31, -47));
	// RclMain.updateBotPose(&left_rcl);
	// liftTarget = 45000;
	// delay(500);
	// claw.move(-127);
	// delay(200);

	// // matchload stack #3 -> goal #2
	// chassis.moveToPoint(-47, -58.5, 1000, {.maxSpeed = 90});
	// chassis.turnToHeading(90, 1000, {.maxSpeed = 90});
	// claw.move(127);
	// liftTarget = -2000;
	// chassis.moveToPoint(-62, -58.5, 1500, {.forwards = false, .maxSpeed = 40});
	// chassis.moveToPoint(-47, -47, 1200, {.maxSpeed = 50});
	// delay(300);
	// liftTarget = 100000;
	// chassis.turnToHeading(270, 1500, {.maxSpeed = 60});
	// chassis.moveToPoint(-31, -47, 1300, {.forwards = false, .maxSpeed = 70});
	// chassis.waitUntilDone();
	// RclMain.setRclPose(Pose(-31, -47));
	// RclMain.updateBotPose(&left_rcl);
	// liftTarget = 80000;
	// delay(500);
	// claw.move(-127);
	// delay(200);

	// // matchload stack #1 -> goal #1
	// chassis.moveToPoint(-47, -58.5, 1000, {.maxSpeed = 90});
	// chassis.turnToHeading(90, 1000, {.maxSpeed = 90});
	// claw.move(127);
	// liftTarget = -2000;
	// chassis.moveToPoint(-62, -58.5, 1500, {.forwards = false, .maxSpeed = 40});
	// chassis.moveToPoint(-47, -47, 1200, {.maxSpeed = 50});
	// delay(300);
	// liftTarget = 100000;
	// chassis.turnToHeading(180, 1500, {.maxSpeed = 60});
	// chassis.moveToPoint(-47, -31, 1300, {.forwards = false, .maxSpeed = 70});
	// chassis.waitUntilDone();
	// RclMain.setRclPose(Pose(-47, -31));
	// RclMain.updateBotPose(&left_rcl);
	// liftTarget = 80000;
	// delay(500);
	// claw.move(-127);
	// delay(200);
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