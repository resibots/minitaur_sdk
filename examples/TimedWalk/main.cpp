/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>, Avik De <avik@ghostrobotics.io>, and Turner Topping <turner@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
#endif

// Seconds passed
int timer = 0;

// Previous time in milliseconds
uint32_t oldMillis = S->millis;

/* This example uses the higher-level Peripheral rather than a Behavior,
*  so that it is able to make use of the Minitaur's built in walk Behavior.
*  Only one Behavior can run at a time, and any number of Peripherals can
*  run concurrently with the currently running Behavior, so we use this to:
* 
*  - Stand for five seconds
*  - Walk forward for five seconds
*  - Repeat
* 
*  See the "TimedWalk" example in the Ghost Robotics SDK for a walkthrough:
*  http://ghostrobotics.gitlab.io/SDK/index.html
*/ 
class TimedWalk : public Peripheral
{
public:

	void begin()
	{
		// Command by behavior
		C->mode = RobotCommand_Mode_BEHAVIOR;

		// Start behavior
		C->behavior.mode = BehaviorMode_RUN;

		// Stand still
		C->behavior.twist.linear.x = 0.0;

		// Save time
		oldMillis = S->millis;
	}

	void update()
	{
		// Increase timer
		if(S->millis > oldMillis + 1000) 
		{
			timer++;
			oldMillis = S->millis;
		}

		// Choose behavior speed
		if(timer < 5){
			// Stand still
			C->behavior.twist.linear.x = 0.0;
		}
		else if(timer >= 5 && timer < 10)
		{
			// Walk forward slowly
			C->behavior.twist.linear.x = 0.2;
		}
		else if(timer >= 10) {
			// Reset timer
			timer = 0;
		}
	}
};

void debug()
{
	printf("Timer: %d\r\n", timer);
}

int main(int argc, char *argv[])
{
#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motZeros[i];
#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
#else
#error "Define robot type in preprocessor"
#endif

	// Disable joystick input
	JoyType joyType = JoyType_NONE;
	ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);

	// Create controller peripheral
	TimedWalk timedWalk;
	timedWalk.begin();

	// Add it
	addPeripheral(&timedWalk);

	// Remove bound behavior from Minitaur (first element of behaviors vector), so we're left with only walk behavior
	behaviors.erase(behaviors.begin());

	return begin();
}
