/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Raj Patel <raj.patel@ghostrobotics.io>, Tom Jacobs <tom.jacobs@ghostrobotics.io>, and Avik De <avik@ghostrobotics.io>
 */

/**
 * See "Programming Overview" in the Ghost Robotics SDK to learn about creating your own behavior.
 * http://ghostrobotics.gitlab.io/SDK/MinitaurProgrammingOverview.html
 */

#include <stdio.h>
#include <SDK.h>
#include <Motor.h>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
#endif

class Starter : public Behavior {
public:
	// begin() is called once when the behavior starts
	void begin() {

		// Command limbs
		C->mode = RobotCommand_Mode_LIMB;

	}

	// signal() is called when receiving a signal from the controller
	// void signal(uint32_t sig) {

	// }

	// update() is called once per loop while the behavior is running
	void update() {

		// Starter when RUN
		if (C->behavior.mode == BehaviorMode_RUN)
		{

		}
		// Relax when STOP
		else if (C->behavior.mode == BehaviorMode_STOP)
		{
			// Calculate leg angle value
			// Use the negative pitch of the robot to keep the leg pointing down
			float angle = -S->imu.euler.y;

			// For all legs
			for (int i = 0; i < P->limbs_count; ++i)
			{
				// Set limb type
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

				// Set leg angle gain and positon
				limb[i].setGain(ANGLE, 0.8, 0.03);
				limb[i].setPosition(ANGLE, angle);

				// Set leg extension gain and position
				limb[i].setGain(EXTENSION, 120, 4);
				limb[i].setPosition(EXTENSION, 0.15);
			}
		}

	}

	// running() is called to determine whether the behavior is running or not
	// void running() {

	// }

	// end() is called when the behavior is stopped
	void end() {

	}
};

// Main
int main(int argc, char *argv[]) {
#if defined(ROBOT_MINITAUR)
	// Create MINITAUR
	init(RobotParams_Type_MINITAUR, argc, argv);

	// Set motor zeros
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i];
#elif defined(ROBOT_MINITAUR_E)

	// Create MINITAUR_E
	init(RobotParams_Type_MINITAUR_E, argc, argv);

	// Set joystick type
	JoyType joyType = JoyType_FRSKY_XSR;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
#else
#error "Define robot type in preprocessor"
#endif

	// Uncomment to clear Bound and Walk behaviors
	// behaviors.clear();

	// Create, add, and start Starter behavior
	Starter starter;
	behaviors.push_back(&starter);
	starter.begin();

	// Run
	return begin();
}

// debug() is called at the DEBUG_RATE
void debug() {

}
