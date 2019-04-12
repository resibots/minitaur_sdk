/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Raj Patel <raj.patel@ghostrobotics.io>, Tom Jacobs <tom.jacobs@ghostrobotics.io>, and Avik De <avik@ghostrobotics.io>
 */

/**
 * Purpose:
 * 		This example will show the benefits of
 *			(1) setting a timer,
 * 			(2) strengthening the leg angle and extension gains,
 *			(3) changing the leg angle and extension positions,
 * 			(4) and using the weight of the robot.
 *
 * Functionalities:
 * 		When STOP:
 * 			The robot will change its height, pitch, and roll with changes in
 * 			the left vertical axis and right axes of the controller.
 *
 * 		When RUN:
 * 			The robot will tripod once by lifting the leg in the direction of
 *			the right axes of the controller, hold for 2 s, and then relax.
 *
 * See "Example: RockNRoll" in the Ghost Robotics SDK for a walkthrough.
 * http://ghostrobotics.gitlab.io/SDK/RockNRoll.html
 */

#include <stdio.h>
#include <SDK.h>
#include <Motor.h>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
#endif

#define MIN_3AXISRANGE -3.0
#define MAX_3AXISRANGE 3.0
#define MIN_EXTENSION 0.10
#define HALF_EXTENSION 0.15
#define MAX_EXTENSION 0.20

class RockNRoll : public Behavior {
public:
	// Variable for loop time in ms
	uint32_t loopTime;

	// Variable for every 100 ms passed
	int timer = 0;

	// Variable for indicating whether the robot is tripoding or not
	bool tripoding = false;

	// begin() is called once when the behavior starts
	void begin() {

		// Command limbs
		C->mode = RobotCommand_Mode_LIMB;

		// Save first loop time
		loopTime = S->millis;

	}

	// update() is called once per loop while the behavior is running
	void update() {

		// Case: 100 ms passed since previous saved loop time
		if(S->millis >= loopTime + 100)
		{
			// Increment for 100 ms passed
			timer++;

			// Save current loop time
			loopTime = S->millis;
		}

		// Calculate leg angle position
		// Use the negative pitch of the robot to keep the leg pointing down
		// Add calibration to bring toe under motors
		float angle = -S->imu.euler.y + 0.1;

		// For all legs
		for (int i = 0; i < P->limbs_count; ++i)
		{
			// Set limb type
			P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

			// Set leg angle gain and position
			limb[i].setGain(ANGLE, 1.0, 0.03);
			limb[i].setPosition(ANGLE, angle);

			// Set leg extension gain
			limb[i].setGain(EXTENSION, 140, 4);
		}

		// Case: Tilt when STOP
		if (C->behavior.mode == BehaviorMode_STOP)
		{
			// Change the height of the robot with the left vertical axis of the controller
			float height = S->joy.axes[2];

			// Change the pitch of the robot with the right vertical axis of the controller
			float pitch = S->joy.axes[1];

			// Change the roll of the robot with the right horizontal axis of the controller
			float roll = S->joy.axes[0];

			// Calculate and set first leg extension position
			float extension = map(height - pitch + roll, MIN_3AXISRANGE, MAX_3AXISRANGE, MIN_EXTENSION, MAX_EXTENSION);
			limb[0].setPosition(EXTENSION, extension);

			// Calculate and set second leg extension position
			extension = map(height + pitch + roll, MIN_3AXISRANGE, MAX_3AXISRANGE, MIN_EXTENSION, MAX_EXTENSION);
			limb[1].setPosition(EXTENSION, extension);

			// Calculate and set third leg extension position
			extension = map(height - pitch - roll, MIN_3AXISRANGE, MAX_3AXISRANGE, MIN_EXTENSION, MAX_EXTENSION);
			limb[2].setPosition(EXTENSION, extension);

			// Calculate and set fourth leg extension position
			extension = map(height + pitch - roll, MIN_3AXISRANGE, MAX_3AXISRANGE, MIN_EXTENSION, MAX_EXTENSION);
			limb[3].setPosition(EXTENSION, extension);

			// Set to not tripoding
			tripoding = false;
			timer = 0;
		}
		// Case: Tripod once when RUN
		else if (!tripoding)
		{
			// Case: Lift first leg when the right axes of the controller point top left
			if (S->joy.axes[1] > 0 && S->joy.axes[0] < 0)
			{
				limb[0].setPosition(EXTENSION, HALF_EXTENSION);

				limb[1].setPosition(EXTENSION, MAX_EXTENSION);

				limb[2].setPosition(EXTENSION, MAX_EXTENSION);

				limb[3].setPosition(EXTENSION, MIN_EXTENSION);
			}
			// Case: Lift second leg when the right axes of the controller point bottom left
			else if (S->joy.axes[1] < 0 && S->joy.axes[0] < 0)
			{
				limb[0].setPosition(EXTENSION, MAX_EXTENSION);

				limb[1].setPosition(EXTENSION, HALF_EXTENSION);

				limb[2].setPosition(EXTENSION, MIN_EXTENSION);

				limb[3].setPosition(EXTENSION, MAX_EXTENSION);
			}
			// Case: Lift third leg when the right axes of the controller point top right
			else if (S->joy.axes[1] > 0 && S->joy.axes[0] > 0)
			{
				limb[0].setPosition(EXTENSION, MAX_EXTENSION);

				limb[1].setPosition(EXTENSION, MIN_EXTENSION);

				limb[2].setPosition(EXTENSION, HALF_EXTENSION);

				limb[3].setPosition(EXTENSION, MAX_EXTENSION);
			}
			// Case: Lift fourth leg when the right axes of the controller point bottom right
			else
			{
				limb[0].setPosition(EXTENSION, MIN_EXTENSION);

				limb[1].setPosition(EXTENSION, MAX_EXTENSION);

				limb[2].setPosition(EXTENSION, MAX_EXTENSION);

				limb[3].setPosition(EXTENSION, HALF_EXTENSION);
			}

			// Set to tripoding
			tripoding = true;
		}
		// Case: Relax from tripoding after 2 s
		else if (timer > 20)
		{
			for (int i = 0; i < P->limbs_count; ++i)
				limb[i].setPosition(EXTENSION, HALF_EXTENSION);
		}

	}

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

	// Clear Bound and Walk behaviors
	behaviors.clear();

	// Create, add, and start RockNRoll behavior
	RockNRoll rocknroll;
	behaviors.push_back(&rocknroll);
	rocknroll.begin();

	// Run
	return begin();
}

// debug() is called at the DEBUG_RATE
void debug() {

}
