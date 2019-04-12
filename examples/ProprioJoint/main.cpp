/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Gavin Kenneally, Avik De, Turner Topping <gavin@ghostrobotics.io> <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <Smath.h>
#include <Motor.h>
#include <Behavior.h>



/**
 * State machine representation:
 * PJ_CW -> Clockwise
 * PJ_CCW -> Counter-Clockwise
 */
enum PJMode {
	PJ_CW = 0, PJ_CCW
};

/**
 * See "Joint control: the ProprioJoint behavior" in the documentation for a walk-through
 * guide.
 */
class ProprioJoint : public Behavior {
public:
	PJMode mode; //Current state within state-machine

	float posDes; //Desired position
	float torqueThres = 0.1; //Torque threshold that triggers state change

	uint32_t tLast; // System time @ last state change
	
	void begin() {
		mode = PJ_CW; //Start in clockwise mode
		posDes = 0; // Initialize desired position to zero
		tLast = S->millis;// Set tLast at onset 
	}

	void update() {
		C->mode = RobotCommand_Mode_JOINT;

		//Trigger state change if torque is higher then threshold and if it has been 
		// at least 1 second since last state change.
		if (fabsf(joint[0].getTorqueEst()) > torqueThres && (S->millis - tLast) > 1000) {
			mode = (mode == PJ_CW ? PJ_CCW : PJ_CW);
			tLast = S->millis; //Update time of last state change
		}

		joint[0].setGain(0.7, 0.006); //Sets P and D gains for position based control
		
		if (mode == PJ_CCW) {
			//If CCW, the desired position is update by fixed increment in CCW direction
			posDes = posDes + 0.00314;
		} else if(mode == PJ_CW) {
			//If CW, desired position is updated in CW direction
		 	posDes = posDes - 0.00314;
		}
		joint[0].setPosition(posDes);//Set motor position to newly incremented desired position.
	}

	bool running() {
		return true;
	}

	void end() {
	}
};

int main(int argc, char *argv[]) {
	init(RobotParams_Type_MINITAUR, argc, argv);
	ProprioJoint proprioJoint; //Declare instance of our behavior
	//Disable the safety shut off feature:
	//IT IS POSSIBLE TO DAMAGE THE MOTOR; BE CAREFUL WHEN USING
	//BEHAVIORS WITHOUT THIS FAILSAFE 
	safetyShutoffEnable(false);
	//Disable the softStart feature
	softStartEnable(false);
	//Remove default behaviors from behaviors vector
	behaviors.clear();
	//add our behavior to behaviors vector
	behaviors.push_back(&proprioJoint);

	return begin();
}

