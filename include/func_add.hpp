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
                liftTarget = -200;
            } else if (level == 1) {
                liftTarget = 27000;
            } else if (level == 2) {
                liftTarget = 48000;
            } else if (level == 3) {
                liftTarget = 73000;
            } else if (level == 4) {
                liftTarget = 98000;
            }
            
            liftError = liftTarget - lift_rotation.get_position();
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
        } else {
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