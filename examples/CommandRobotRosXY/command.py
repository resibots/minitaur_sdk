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
import numpy as np

# Import our ROS interfacing library
import ros_interface

#this function is computing a circular trajectory
def circle(omega,radius,t):
	x = radius*np.cos(omega*t)+0.14
	y = radius*np.sin(omega*t)
	return x,y

#Compute checksum to check message completeness
def calculateChecksum(bytes, length):
	checksum = ct.c_ushort(0)
	for i in range(length):
		checksum = ct.c_ushort(int(checksum.value) + ord(bytes[i]));
	return checksum.value

# Open USB port
portAddress = "/dev/ttyUSB0"
#Set baud rate
baud = 115200

# No timeout so we can keep going with whatever bytes are in waiting
port = serial.Serial(portAddress, baud, timeout=None)
print("Sending to: " + port.name)

# Receive buffer
rxBuf = ''

# Test by commanding forwards and backwards
phase = 0.0


# Create ROS publishers and subscribers if ROS is present
ros_interface.initROS()

t = 0;
dt = 0.01;#used to compute trajectory

while ros_interface.toContinue():
	sleep(0.001) # 1ms

	x,y = circle(2,0.01,t)
	t=t+dt

	# If ROS, get commands, otherwise, just walk forwards and backwards
	if ros_interface.bROS:

		# Init
		linear = [0,0,0]
		angular = [0,0,0]
		position = [0,0,0]
		orientation = [0,0,0,0]


		# Our commands
		behaviorId = 1
		behaviorMode = 0
		height = 0
		lateral = 0
		X, Y, linear, angular, position, orientation, behaviorId, behaviorMode, height, lateral, restart = ros_interface.getCommands()
		# x=0.14
		# X=[x,x,x,x]
		# y=0.05
		# Y=[y,y,y,y]
		xycommand = struct.pack( '<I4f4f1?', 1,X[0],X[1],X[2],X[3],Y[0],Y[1],Y[2],Y[3], restart)

		# Calculate and append checksum, prepend align bytes
		checksum = calculateChecksum(xycommand, len(xycommand))
		xycommand = struct.pack('<cc', b"G", b"R") + xycommand + struct.pack('<H', checksum)

		# Send BehaviorCmd via USB
		port.write(xycommand)

		# Read incoming data
		if True:
			rxBuf += port.read(50)
			chunks = rxBuf.split('GR')
			for chunk in chunks:
				if len(chunk) > 0:
					try:
						# Parse packet into tuple: time, lastRX time, 3 float imu state, 8 float joint pos, 8 float joint vel, 8 float joint torque, checksum
						tup = struct.unpack('<2I3f8f8f8f8f8f1?H', chunk)
					except:
						continue

					# Get times
					time = tup[0]
					lastRXtime = tup[1]
					# Compare checksum
					checksum = tup[len(tup)-1]
					checksumCalculated = calculateChecksum('GR' + chunk, 2+4+4+(3*4)+5*4*8 + 1)
					# # Print state
					if(checksum == checksumCalculated):
						ros_interface.publishStateRos(tup, time)

			# Put back the last (possibly unfinished) chunk
			rxBuf = chunks[-1]

		# Increase phase
		phase += 1


# Close
port.close()
