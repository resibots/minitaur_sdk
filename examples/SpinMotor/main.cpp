/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io> and Tom Jacobs <tom.jacobs@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <Motor.h>

class Test : public Behavior
{
public:
	void begin()
	{

	}

	void update()
	{
		// Control robot by joints
		C->mode = RobotCommand_Mode_JOINT;

		// Spin all motors
		for (int i = 0; i < 8; i++)
			joint[i].setOpenLoop(0.15);
	}

	void end()
	{

	}
};

void debug()
{
	// If motor has a temperature sensor installed, print temperature
	for(int i = 0; i < 8; i++)
		if( joint[i].getTemperature() != 0 )
		printf("Temperature of motor %d: %.0f degrees C\n", i, joint[i].getTemperature());
}

int main(int argc, char *argv[])
{
	// Loads some default settings including motor parameters
#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);
#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
#else
#error "Define robot type in preprocessor"
#endif

	// Configure joints
	#define NUM_MOTORS 8
	const float zeros[NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0};
	const float directions[NUM_MOTORS] = {1, 1, 1, 1, 1, 1, 1, 1};
	P->joints_count = S->joints_count = C->joints_count = NUM_MOTORS;
	for (int i = 0; i < P->joints_count; i++)
	{
		// Set zeros and directions
		P->joints[i].zero = zeros[i];
		P->joints[i].direction = directions[i];
	}

	// No limbs, only 1DOF joints
	P->limbs_count = 0; 

	// Remove default behaviors from behaviors vector, create, add, and start ours
	Test test;
	behaviors.clear();
	behaviors.push_back(&test);
	test.begin();

	return begin();
}
