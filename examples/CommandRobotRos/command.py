#
# Copyright (C) Ghost Robotics - All Rights Reserved
# Unauthorized copying of this file, via any medium is strictly prohibited
# Proprietary and confidential
# Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>, Avik De <avik@ghostrobotics.io>
#
# Command: Sends BehaviorCmds to the robot to command its behavior.

import struct
import serial
import math
from time import sleep
import ctypes as ct

# Import our ROS interfacing library
import ros_interface

# Open USB port
portAddress = "/dev/ttyUSB0"
#portAddress = "/dev/tty.usbserial-DN01QAKV"
#portAddress = "COM29"
baud = 115200
# baud = 230400
# No timeout so we can keep going with whatever bytes are in waiting
port = serial.Serial(portAddress, baud, timeout=None)
print("Sending to: " + port.name)

# Receive buffer
rxBuf = ''

# Test by commanding forwards and backwards
phase = 0.0


# Create ROS publishers and subscribers if ROS is present
ros_interface.initROS()

def calculateChecksum(bytes, length):
	checksum = ct.c_ushort(0)
	for i in range(length):
		checksum = ct.c_ushort(int(checksum.value) + ord(bytes[i]));
	return checksum.value

while ros_interface.toContinue():
	sleep(0.001) # 1ms

	# If ROS, get commands, otherwise, just walk forwards and backwards
	if ros_interface.bROS:

		# Init
		linear = [0,0,0]
		angular = [0,0,0]
		position = [0,0,0]
		orientation = [0,0,0,0]

		# Our commands
		linear_x = 0
		angular_z = 0
		behaviorId = 1
		behaviorMode = 0
		height = 0
		lateral = 0
		linear, angular, position, orientation, behaviorId, behaviorMode, height, lateral = ros_interface.getCommands()

		# Set values
		id = 0
		mode = 1                                  # BehaviorMode_RUN
		behavior_command = struct.pack( '<II3f3f3f4fI', 1, # Version 1 of serial packet format
														id,
														linear[0], linear[1], linear[2],
														angular[0], angular[1], angular[2],
														position[0], position[1], position[2],
														0,0,0,0,
														mode)

		# Calculate and append checksum, prepend align bytes
		checksum = calculateChecksum(behavior_command, len(behavior_command))
		behavior_command = struct.pack('<cc', b"G", b"R") + behavior_command + struct.pack('<H', checksum)

		# Send BehaviorCmd via USB
		port.write(behavior_command)

		# Read incoming data
		if True:
			rxBuf += port.read(50)
			chunks = rxBuf.split('GR')
			for chunk in chunks:
				if len(chunk) > 0:
					try:
						# Parse packet into tuple: time, lastRX time, 3 float imu state, 8 float joint pos, 8 float joint vel, 8 float joint torque, checksum
						tup = struct.unpack('<2I3f8f8f8fH', chunk)
					except:
						continue

					# Get times
					time = tup[0]
					lastRXtime = tup[1]
					# Compare checksum
					checksum = tup[len(tup)-1]
					checksumCalculated = calculateChecksum('GR' + chunk, 2+4+4+(3*4)+3*4*8)
					# Print state
					if(checksum == checksumCalculated):
						print tup
						ros_interface.publishStateRos(tup, time)

			# Put back the last (possibly unfinished) chunk
			rxBuf = chunks[-1]

		# Increase phase
		phase += 1


# Close
port.close()


# BehaviorCmd definition:

# typedef struct _BehaviorCmd {
#     uint32_t id;
#     Twist twist;
#     Pose pose;
#     uint32_t mode;
# } BehaviorCmd;

# typedef struct _Twist {
#     Vector3 linear;
#     Vector3 angular;
# } Twist;

# typedef struct _Pose {
#     Vector3 position;
#     Quaternion orientation;
# } Pose;

# typedef struct _Vector3 {
#     float x;
#     float y;
#     float z;
# } Vector3;

# typedef struct _Quaternion {
#     float x;
#     float y;
#     float z;
#     float w;
# } Quaternion;
