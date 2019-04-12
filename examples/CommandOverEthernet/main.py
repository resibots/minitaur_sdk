'''
MIT License (modified)

Copyright (c) 2018 Ghost Robotics
Authors:
Avik De <avik@ghostrobotics.io>
Tom Jacobs <tom.jacobs@ghostrobotics.io>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this **file** (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

''' How to use:

## Hardware setup

Note: The computer must have a dedicated ethernet port, and its address must be configured as follows (in `/etc/network/interfaces`):

auto <ethX>
iface <ethX> inet static
address 169.254.98.1
netmask 255.255.255.0

where <ethX> is the name of the ethernet interface. Running `$ ifconfig <ethX>` must return (if it doesn't, restart the computer after changing `/etc/network/interfaces`)

... inet addr:169.254.98.1 ...

(The mainboard does not have a full TCP/IP stack with ARP, so the address must match)

## Running this script

Again, due to the lack of a full TCP/IP stack on the mainboard, this script must be bound to <ethX>, and that operation is only allowed for root.

* Run `$ sudo -s`
* Run `# python udp_comms.py <ethX>`

If will print received data to terminal, and if ROS is found, also publish to ROS at a decimated rate

'''

import numpy as np
import time
import argparse
import struct

# Import our ethernet sending receiving library
import ethernet

# Import our ROS interfacing library
import ros_interface

# Our commands
linear_x = 0
angular_z = 0
behaviorId = 0
behaviorMode = 1
height = 0
lateral = 0
tstart = time.time()

# Main
if __name__ == "__main__":

	# Parse args
	parser = argparse.ArgumentParser(usage= 'Due to the lack of a full TCP/IP stack on the mainboard, this script must be bound to <iface>, and that operation is only allowed for root. \n\n\
* Run `$ sudo -s` \n\
* Run `# python udp_comms.py <iface>` \n\n\
It will print received data to terminal, and if ROS is found, also publish to ROS at a decimated rate.')
	parser.add_argument('iface', metavar='iface', type=str, nargs='?', help='name of the network interface; e.g. eth0', default='enxe4b97a92cc38')
	parser.add_argument('--ros-pub-dec', metavar='N', type=int, nargs=1, help='ROS publish decimation rate', default=10)
	args = parser.parse_args()

	# Open RX socket
	srx = ethernet.openRXSocket()

	# Set non-blocking receive so we get the freshest packet
	srx.setblocking(1)


	# Open TX socket
	stx = ethernet.openTXSocket(args.iface)
	print"ok"
	state, numDoF = ethernet.receivePacket(stx)
	print "pas ok"
	

	# Create ROS publishers and subscribers if ROS is present
	ros_interface.initROS()

	# Loop
	print("Starting ethernet RX and TX.")
	while ros_interface.toContinue():
		time.sleep(0.001) # 1ms

		# If ROS, get commands, otherwise, just walk forwards and backwards
		if ros_interface.bROS:
			linear_x, angular_z, behaviorId, behaviorMode, height, lateral = ros_interface.getCommands()
			# height = 0.8
			# behaviorMode = 1
			linear_x = 0.0
			behaviorMode = 1
			behaviorId = 0
			# height = 0.2
			# angular_z = 0.7
			# print linear_x
			# angular_z = 0.6
		else:
			print("demo")
			#Do a scripted demo
			if time.time() < tstart + 5:
				print("Up")
				height = 0.8
			elif time.time() < tstart + 10:
				print("Step")
				height = 0.8
				behaviorMode = 1
				linear_x = 0.2
			elif time.time() < tstart + 12:
				print("Yaw")
				height = 0.8
				behaviorMode = 1
				linear_x = 0
				angular_z = 0.6
			elif time.time() < tstart + 14:
				print("Yaw back")
				height = 0.8
				behaviorMode = 1
				linear_x = 0
				angular_z = -0.6
			elif time.time() < tstart + 16:
				print("Step back")
				height = 0.8
				angular_z = 0
				behaviorMode = 1
				linear_x = -0.4
			elif time.time() < tstart + 18:
				print("Down")
				behaviorMode = 0
				height = 0.3

		# Cap values
		linear_x = max(min(linear_x, 1), -1)
		angular_z = max(min(angular_z, 1), -1)
		lateral = max(min(lateral, 1), -1)

		# Pack BehaviorCmd (corresponding parsing code is in main.cpp)
		# BehaviorCmd = behaviorId, twist, pose, behaviorMode
		# print behaviorId
		# print behaviorMode
		# print linear_x
		# print height
		behaviorCmd = struct.pack('<I6f7fI',
			behaviorId,
			linear_x, 0, 0,
			0, 0, angular_z,
			0, lateral, height, 0, 0, 0, 0,
			behaviorMode)

		# TX
		# print("Sending linear.x: " + str(linear_x) + " angular.z: " + str(angular_z) + " height: " + str(height) + " id: " + str(behaviorId) + " mode: " + str(behaviorMode))
		try:
			ethernet.transmitPacket(stx, behaviorCmd)
		except:
			pass

		# RX
		try:
			state, numDoF = ethernet.receivePacket(srx)
			# print("state ", state)
			if not state: continue
		except:
			continue

		# Print robot state
		# print(state['millis'], state['battery/voltage'])
		# print(state['millis'], state['userData'])
		#print(state['millis'], state['joint/position'][:8])
		#print(str(state['millis']), str(state['imu/euler']))

		# Publish robot state to ROS
		ros_interface.publishState(state, args.ros_pub_dec, numDoF)
