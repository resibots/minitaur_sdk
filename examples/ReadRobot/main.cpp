/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>
 */
#include <stdio.h>
#include <SDK.h>
#include <unistd.h>

#if defined(ROBOT_MINITAUR)
// Subject to change for individual robots
const float motorZeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};
#endif

typedef struct OutStruct {
    uint32_t alignment = 0xffffffff;
    uint32_t millis;
    uint16_t position;
    uint16_t velocity;
} __attribute__((packed)) OutStruct;
OutStruct outs;

class ReadRobot: public Peripheral
{
public:

	void begin()
	{

	}

	void update()
	{
		// Populate a struct and write the data to the USB port
		outs.millis = S->millis;
	    outs.position = int(10000 * (3.1415 + S->joints[0].position)); // Write encoder value, to range between 0 to 62830
		outs.velocity = int(32768 + 100 * S->joints[0].velocity);      // Write velocity value (100X), to range 0 to 65535, where 32768 is zero
		write(STDOUT_FILENO, (char *)&outs, sizeof(outs));
	}
};

int main(int argc, char *argv[])
{
#if defined(ROBOT_MINITAUR)
	init(RobotParams_Type_MINITAUR, argc, argv);
	for (int i = 0; i < P->joints_count; ++i)
		P->joints[i].zero = motorZeros[i];
#elif defined(ROBOT_MINITAUR_E)
	init(RobotParams_Type_MINITAUR_E, argc, argv);
#else
#error "Define robot type in preprocessor"
#endif

	// Set USB to baud rate: 230400
	// Potential baud rates: 57600, 115200, 230400, 460800, 921600
	SerialPortConfig cfg;
	cfg.baud = 230400;
	cfg.mode = SERIAL_8N1;
	ioctl(STDOUT_FILENO, IOCTL_CMD_SERIAL_PORT_CFG, &cfg);

	// Create controller peripheral
	ReadRobot readRobot;
	readRobot.begin();

	// Add it
	addPeripheral(&readRobot);

	// Go
	return begin();
}
