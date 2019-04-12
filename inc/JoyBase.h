/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */

#ifndef JoyBase_h
#define JoyBase_h

#include "Peripheral.h"
#include "simple.pb.h"
// #include "SDK.h"

//extern Behavior *behavior;
//
//// behavior array: add behaviors here
//extern const int NUM_BEHAVIORS;
//extern Behavior *behaviorArray[];
//
//// function to programmatically change behavior
//void activateBehavior(Behavior *behav);
//
//extern Peripheral *remote;// defined in .ino file

/**
	 * @brief Updates a float to desired within rate limits
	 * 
	 * @param current state to update
	 * @param desired new desired
	 * @param rateLimitOneStep limit in units of /step
	 */
float rateLimitedUpdate(float &current, const float &desired, float rateLimitOneStep);

class JoyBase : public Peripheral {
protected:
	float JOY_SPEED_SENS = 1.0;//m/s. 0.6 (low), 0.8 (medium), 1.5 (high)
	float JOY_YAW_SENS = 1.2;//rad/s.

	float JOY_SMOOTH_TWIST_LINEAR = 1.0;
	float JOY_SMOOTH_TWIST_ANGULAR = 0.5;
	float JOY_SMOOTH_TWIST_POSE_Z = 0.9;
	// rate limiter, in units of /s; i.e. if twist.linear, it will be an acceleration limit in m/s^2
	float JOY_RATE_LIMIT_TWIST_LINEAR = 1.0;
	float JOY_RATE_LIMIT_TWIST_ANGULAR = 2.0;

public:
	JoyType type = (JoyType)0;
	// From peripheral
	virtual void begin() = 0;
	virtual void update() = 0;
	// New for JoyBase
	virtual void toBehaviorCmd(BehaviorCmd *b) = 0;

	/**
	 * @brief Set joystick sensitivity
	 * 
	 * @param speedSens Joystick axis at max range maps to +/- speedSens m/s
	 * @param yawSens Joystick axis at max range maps to +/- yawSens rad/s
	 */
	void setSensitivity(float speedSens, float yawSens) {
		JOY_SPEED_SENS = speedSens;
		JOY_YAW_SENS = yawSens;
	}

	virtual ~JoyBase() {}
};


#endif
