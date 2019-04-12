/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>, Avik De <avik@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <unistd.h>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
const float motZeros[8] = {2.82, 3.435, 3.54, 3.076, 1.03, 3.08, 6.190, 1.493};
#endif

/* 
*
*  Command Robot
*  
*  This example shows you how to command the robot's behavior using a USB 
*  serial link.
*
*  This example uses the higher-level Peripheral rather than a Behavior,
*  so that it is able to make use of the Minitaur's built in walk Behavior.
*  Only one Behavior can run at a time, and any number of Peripherals can
*  run concurrently with the currently running Behavior, so we use this to
*  control the robot by commanding the built in walk behavior.
* 
*  See the SDK docs for more information:
*  http://ghostrobotics.gitlab.io/SDK/index.html
* 
*/

#pragma pack(push, 1)
// We receive the serial protocol version number (in case we add more fields later)
// our behaviorCmd, and a checksum.
struct SerialCommandPacket
{
	uint32_t version;
	BehaviorCmd behavior_command;
	uint16_t checksum;
};

const char ALIGNMENT_WORD[2] = {'G', 'R'};

struct SerialStatePacket
{
	char align[2];
	uint32_t millis, lastRX; // report last reception from the computer
	Vector3 euler; // some robot state
	uint16_t checksum;
};
#pragma pack(pop)

class CommandRobot : public Peripheral
{
public:
	void begin()
	{
		// Command by behavior
		C->mode = RobotCommand_Mode_BEHAVIOR;
	}

	// Parser state
	// Goes from 0 to 1 to 2
	int numAlignmentSeen = 0;
	uint16_t rxPtr = 0;

	// Receive buffer and alignment
	const static uint16_t RX_BUF_SIZE = 100;

	SerialCommandPacket commandPacket;
	SerialStatePacket statePacket;

	// Keep track of our last state packet send
	const static uint32_t TX_EVERY_MS = 10;
	uint32_t lastTX = 0;

	// Helper function to calculate a simple sum-of-bytes checksum
	uint16_t bufChecksum(const uint8_t *buffer, uint16_t nbytes)
	{
		uint16_t sum = 0;
		for (uint16_t i = 0; i < nbytes; ++i)
		{
			sum += buffer[i];
		}
		return sum;
	}

	void update()
	{
		// Character to store the latest received character
		uint8_t latestRX;

		// Loop through while there are new bytes available
		while (read(STDIN_FILENO, &latestRX, 1) > 0)
		{
			if (numAlignmentSeen == 0 && latestRX == ALIGNMENT_WORD[0])
			{
				numAlignmentSeen = 1;
			}
			else if (numAlignmentSeen == 1 && latestRX == ALIGNMENT_WORD[1])
			{
				numAlignmentSeen = 2;
				rxPtr = 0;
			}
			else if (numAlignmentSeen == 2)
			{
				// Add the next byte to our memory space in serial_packet
				uint8_t *pSerialPacket = (uint8_t *)&commandPacket;
				pSerialPacket[rxPtr++] = latestRX; // post-increment rxPtr

				// Check if we have read a whole packet
				//printf("ptr %d %d\n", rxPtr, sizeof(SerialPacket));
				if (rxPtr == sizeof(SerialCommandPacket))
				{
					// Check checksum
					uint16_t checksum = bufChecksum(pSerialPacket, sizeof(SerialCommandPacket) - 2);
					if (commandPacket.checksum == checksum)
					{
						// Check we're processing the right version
						if (commandPacket.version == 1)
						{
							// Copy our BehaviorCmd into C
							memcpy(&C->behavior, &commandPacket.behavior_command, sizeof(BehaviorCmd));

							// Store last received time
							statePacket.lastRX = S->millis;
						}
					}
					//printf("Checksum %u %u.\n", checksum, serial_packet.checksum);

					// Reset
					numAlignmentSeen = rxPtr = 0;
				}
			}
		}

		// Send state
		if (S->millis - lastTX > TX_EVERY_MS)
		{
			lastTX = statePacket.millis = S->millis;
			memcpy(statePacket.align, ALIGNMENT_WORD, 2);
			memcpy(&statePacket.euler, &S->imu.euler, sizeof(Vector3));
			statePacket.checksum = bufChecksum((const uint8_t *)&statePacket, sizeof(SerialStatePacket) - 2);
			write(STDOUT_FILENO, &statePacket, sizeof(statePacket));
		}
	}
};

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

	// Change serial port baud speed. 115200, 8N1 is the default already, but this demonstrates how to change to other settings
	SerialPortConfig cfg;
	cfg.baud = 115200;
	// cfg.baud = 230400;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	// Create controller peripheral
	CommandRobot commandRobot;
	commandRobot.begin();

	// Add it
	addPeripheral(&commandRobot);

	// Remove bound behavior from Minitaur (first element of behaviors vector),
	// so we're left with only walk behavior
	behaviors.erase(behaviors.begin());

	// Go
	return begin();
}
