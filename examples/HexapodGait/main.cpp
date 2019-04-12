/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De and Turner Topping <avik@ghostrobotics.io> <turner@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <math.h>
#include <SMath.h>
#include <Motor.h>
#include <Behavior.h>
#include <GaitRunner.h>

const int NMOT = 6; // The number of motors used throughout the behavior(s)
const float zeros[6] = {2.94, 4.032, -1.84, -2.08, 2.74, -1.79}; //Joint Zeros
const float dir[6] = {-1, -1, -1, 1, 1, 1}; // The direction of each motor in the body frame
//State-machine like enum with two states: running and standing
enum HGMode {
	HG_STAND=0, HG_RUN 
};


class HexapodGait : public Behavior {
public:
	HGMode mode = HG_STAND; // Default to STAND MODE
	float gait_speed =  0.0; // Gait-speed parameter, initialized to 0
	float duty[6] =  {0.7,  0.2,  0.2, 0.7, 0.2,  0.2}; //Gait-Runner "duty" parameter
	float sweep[6] = {60, 60, 20, 60, 60,  20}; // Gait-Runner "sweep" parameter
	float offset[6] ={-15,-20, 40,-15, -20, 40}; // Gait-Runner "offset" parameter (negative numbers are counter-clockwise)
	GaitRunner gait; // Declare instance of Gait-Runner
	float legPhase[6]; // Create a "leg-phase" parameter, it will be assigned values later when we call a specific Gait
	float zero_offsets[6] = {0,0,0,0,0,0}; // Gait-Runner "zero offset" parameter
	bool simulate_quad; // Boolean that activates/shuts down middle leg pair, defined at class instantiation. 
	// When hexapod gait is called, we must assign a phase to each leg, and whether or not the middle to legs are active. 
	HexapodGait(float a, float b, float c, float d, float e, float f, bool sq = false) : gait(GaitRunner(duty[0], radians(offset[0]), radians(sweep[0]), gait_speed)) {
		legPhase[0] = a; legPhase[1]=b; legPhase[2]=c; legPhase[3] = d; legPhase[4] = e; legPhase[5] = f;
		simulate_quad = sq;
			}

	void begin() {
		mode = HG_RUN; // begin gait
	}

	void update() {
		C->mode = RobotCommand_Mode_JOINT; //Control robot by joints
		if(mode == HG_STAND){ // When standing
			for(int i = 0; i<NMOT; i++){ // Iterate through each motor
				joint[i].setGain(0.5,.001); //Set a gain
				joint[i].setPosition(0); //Set leg position to zero.
			}
		}else if(mode == HG_RUN){ // When running 
			gait.speed = map(C->behavior.pose.position.z, -1.0, 1.0, 0.0, 2.0); //Map a control input to gait speed and update the parameter
			gait.update(); // update gait runner

			for(int i = 0; i<NMOT; i++){ //Iterate through each leg
				joint[i].setPosition(gait.legClock(legPhase[i] + zero_offsets[i],0)); // Set leg positions according to gait runner
				if(simulate_quad){ //If middle legs are inactive
					joint[1].setPosition(M_PI); //Set them to straight up in the air
					joint[4].setPosition(M_PI);
				}	
			}
		}
	}
	bool running() {
		return !(mode == HG_STAND);
	}

	void end() {
		mode = HG_STAND;// stop gait
	}
};

int main(int argc, char *argv[]) {
	// Loads some default settings including motor parameters and joint types; leave those
	init(RobotParams_Type_MINITAUR, argc, argv);

	// Configure joints
	P->joints_count = S->joints_count = C->joints_count = NMOT;
	for(int i = 0; i<6; i++){
		P->joints[i].zero = zeros[i];
		P->joints[i].direction = dir[i];
	}
	P->limbs_count = 0; // no limbs, only 1DOF joints
	C->mode = RobotCommand_Mode_JOINT; // Only joint control

	//We declare three different definitions of the hexapod gait class:
	HexapodGait quadTrot(0.5, 0.0, 0.0, 0.0, 0.0, 0.5, true); // Quadruped Trot : Front left-back right are a pair as are front right and back left	
	HexapodGait quadBound(0.5, 0.0, 0.0, 0.5, 0.0, 0.0, true); // Quadruped Bound : Front legs and back legs are in pairs
	HexapodGait hexTripod(0.5, 0.0, 0.5, 0.0, 0.5, 0.0); // Alternating tripod : Front and back legs on one side, and the middle leg from the other move together	
	// We don't need any of the Minitaur settings
	safetyShutoffEnable(false);
	softStartEnable(false);
	//Remove default behaviors from behaviors vector
	behaviors.clear();
	//add our behavior to behaviors vector
	behaviors.push_back(&quadTrot);
	behaviors.push_back(&quadBound);
	behaviors.push_back(&hexTripod);

	return begin();
}