#include "pros/misc.h"
#include "subsystem.hpp"

inline int level = 0; /* keeps track of lift's current level */

inline bool manual = false; /* state of control, manual or macroed.
                               when manual, driver will be able
                               to move the lift to whatever degree of movement he needs */

inline bool flipperState = false;
inline int liftTarget;
inline int liftError;
inline float liftkP = 0.02;
inline bool toggleRed = true;

inline void macro_lift() {
	while (true) {
        if (!manual) {
            if (level == 0) {
                liftTarget = -2000;
            } else if (level == 1) {
                liftTarget = 37000;
            } else if (level == 2) {
                liftTarget = 59000;
            } else if (level == 3) {
                liftTarget = 81000;
            } else if (level == 4) {
                liftTarget = 113000;
            }
            
            liftError = liftTarget - lift_rotation.get_position();
            if ((liftTarget - lift_rotation.get_position()) < -80) { // capping negative power to -80 volts
                lift.move(-80);
            }
            lift.move(liftError * liftkP);
        }
		pros::delay(20);
	}
}

inline void macro_toggleIntake() {
    while (true) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-127);
        } else { // auto toggle only works when driver doesnt control
            if (optical.get_proximity() > 240) {
                if (toggleRed == true) {
                    if (optical.get_hue() > 20) {
                        intake.move(-100);
                    } else {
                        intake.move(0);
                    }
                } else if (toggleRed == false) {
                    if (optical.get_hue() < 190) {
                        intake.move(-100);
                    } else {
                        intake.move(0);
                    }
                }
            } else {
                intake.move(0);
            }
        }
        
        pros::delay(20);
    }
}

inline void lift_auto() {
    while (true) {
        liftError = liftTarget - lift_rotation.get_position();
        lift.move(liftError * liftkP);
        pros::delay(20);
    }
}