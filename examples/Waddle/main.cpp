/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Raj Patel <raj.patel@ghostrobotics.io>, Tom Jacobs <tom.jacobs@ghostrobotics.io>, and Avik De <avik@ghostrobotics.io>
 */

/**
 * Purpose:
 * 		This example will show the benefits of:
 * 			(1) setting a timer,
 *      	(2) changing the leg angle and extension positions,
 *			(3) utilizing Reorientable Behavior functions,
 *      	(4) and using the momentum of the robot.
 *
 * Functionalities:
 *      When STOP:
 *      	The robot will relax.
 *
 * 		When RUN:
 * 			The robot will waddle forwards or backwards with a speed set from
 * 			the right vertical axis of the controller.
 *
 * See "Example: Waddle" in the Ghost Robotics SDK for a walkthrough.
 * http://ghostrobotics.gitlab.io/SDK/Waddle.html
 */

#include <stdio.h>
#include <SDK.h>
#include <Motor.h>
#include <ReorientableBehavior.h>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
#endif

#define MIN_1AXISRANGE -1.0
#define MAX_1AXISRANGE 1.0
#define MIN_3AXISRANGE -3.0
#define MAX_3AXISRANGE 3.0
#define MIN_EXTENSION 0.10
#define HALF_EXTENSION 0.15
#define MAX_EXTENSION 0.20

class Waddle : public ReorientableBehavior {
public:
	// Variable for loop time in ms
	uint32_t loopTime;

	// Variable for every 100 ms passed
	int timer = 0;

	// Variable for extension position
	float extension;

	// begin() is called once when the behavior starts
	void begin() {

		// Command limbs
		C->mode = RobotCommand_Mode_LIMB;

		// Save first loop time
		loopTime = S->millis;

	}

	// update() is called once per loop while the behavior is running
	void update() {

		// Case: Waddle when RUN
		if (C->behavior.mode == BehaviorMode_RUN)
		{
			// Case: 100 ms passed since previous saved loop time
			if(S->millis >= loopTime + 100)
			{
				// Increment for 100 ms passed
				timer++;

				// Save current loop time
				loopTime = S->millis;
			}

			// Change the speed of the robot with the right vertical axis of the controller
			float speed = map(C->behavior.twist.linear.x, MIN_1AXISRANGE, MAX_1AXISRANGE, 0, 0.15);

			// Calculate leg angle position
			// Use the negative pitch of the robot to keep the leg pointing down
			float angle = -S->imu.euler.y + speed;

			// For all legs
			for (int i = 0; i < P->limbs_count; ++i)
			{
				// Set limb type
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

				// Set leg angle gain and position
				limb[i].setGain(ANGLE, 0.8, 0.03);
				limb[i].setPosition(ANGLE, angle);

				// Set leg extension gain
				limb[i].setGain(EXTENSION, 120, 4);
			}

			// Case: Lean left for first 300 ms
			if (timer < 3)
			{
				for (int i = 0; i < P->limbs_count; ++i)
				{
					// Case: Calculate leg extension position when right leg
					if (isRight(i))
						extension = map(MAX_1AXISRANGE, MIN_3AXISRANGE, MAX_3AXISRANGE, MIN_EXTENSION, MAX_EXTENSION);
					// Case: Calculate leg extension position when left leg
					else
						extension = map(MIN_1AXISRANGE, MIN_3AXISRANGE, MAX_3AXISRANGE, MIN_EXTENSION, MAX_EXTENSION);

					// Set leg extension position
					limb[i].setPosition(EXTENSION, extension);
				}
			}
			// Case: Lean right for second 300 ms
			else if (timer >= 3 && timer < 6)
			{
				for (int i = 0; i < P->limbs_count; ++i)
				{
					if (isRight(i))
						extension = map(MIN_1AXISRANGE, MIN_3AXISRANGE, MAX_3AXISRANGE, MIN_EXTENSION, MAX_EXTENSION);
					else
						extension = map(MAX_1AXISRANGE, MIN_3AXISRANGE, MAX_3AXISRANGE, MIN_EXTENSION, MAX_EXTENSION);

					limb[i].setPosition(EXTENSION, extension);
				}
			}
			// Case: Lean left again
			else
				timer = 0;
		}
		// Case: Relax when STOP
		else
		{
			float angle = -S->imu.euler.y;

			for (int i = 0; i < P->limbs_count; ++i)
			{
				P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;

				limb[i].setGain(ANGLE, 0.8, 0.03);
				limb[i].setPosition(ANGLE, angle);

				limb[i].setGain(EXTENSION, 120, 4);
				limb[i].setPosition(EXTENSION, HALF_EXTENSION);
			}
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

	// Create, add, and start Waddle behavior
	Waddle waddle;
	behaviors.push_back(&waddle);
	waddle.begin();

	// Run
	return begin();
}

// debug() is called at the DEBUG_RATE
void debug() {

}