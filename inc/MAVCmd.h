/**
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#pragma once

#include <Peripheral.h>
#include <simple.pb.h>

class MAVCmd : public Peripheral
{
// protected:
public:
	bool prevJoyRCEnabled = false;
	bool joyRCEnabled = true;
	BehaviorCmd behaviorCmd; // this is how the computer sends behavior

	/**
	 * @brief Translates to SDK behavior stuff from received MAVLink command. 
	 * 
	 * @param mavId 0 = sit, 1 = stand/look, 2 = walk
	 * @param mavMode boolean (ARM or DISARM)
	 */
	void translateCmd(uint32_t receivedId, uint32_t receivedMode);

	// 
	bool bStandComplete = false;

public:
	// for gain tuning
	bool readGains = false;
	float paramArray[2]; // hardcoded for now
	bool overrideStanceGains = false;
	float channel, data;

	uint32_t ethRxUpdated = 0;
	const uint32_t MAV_MODE_DISARM = 0;
	const uint32_t MAV_MODE_ARM = 1;
	const uint32_t MAV_ID_SIT = 0;
	const uint32_t MAV_ID_STAND = 1;
	const uint32_t MAV_ID_WALK = 2;
	const uint32_t MAV_ID_KILL = 50;
	const uint32_t MAV_ID_FREEZE = 30;
	const uint32_t MAV_ID_SELFCHECK = 60;
	// FIXME replace by looking through behaviors and finding the right one
	const uint32_t ROBOT_ID_SOFTSTART = 0;
	const uint32_t ROBOT_ID_PCWALK = 1;

	bool bEnabled = false;
	// internal state for reporting back to the computer
	uint32_t mavId = 0, mavMode = 0;

	void begin()
	{
		bEnabled = true;
	}

	void update();
};

// Need a single global instance
extern MAVCmd mavCmd;

