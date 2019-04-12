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

import socket
import struct
import zlib
import numpy as np
import os

''' How to use:

See main.py

'''

''' Network defines '''
OBC_IP = '169.254.98.1'
OBC_PORT = 15000
MCU_PORT = 15001
MCU_SRC_PORT = 14999

''' Packet struct definitions '''
GRM_MAX_LENGTH = 600
GRM_MAGIC = 171

# Header
headerFmt = '<BBHiBB'
headerSize = struct.calcsize(headerFmt)

# state1 = discreteState, imu (20f), batteryState (2f), twist (6f), joy (4f4B)
state1Fmt = '<4I20f2f6f4f10B'
state1Size = struct.calcsize(state1Fmt)

# LANI = jointState
LANIFmt = '<2f4H'
LANISize = struct.calcsize(LANIFmt)

# User data (here interpreted as 32 uint32_t's, but the user is free to unpack as they wish, as long as the data size is GRM_USER_DATA_SIZE)
GRM_USER_DATA_SIZE = 32
userDataFmt = '<'+str(GRM_USER_DATA_SIZE)+'B'
userDataSize = struct.calcsize(userDataFmt)

# Robot-specific stuff
class RobotType:
	MINITAUR = 1
	NGR = 2
robotType = RobotType.MINITAUR

''' Helper functions '''
def grmCrc(data):
	# crc32 only works properly on DWORD aligned buffers
	dataSizeMod4 = (len(data) / 4) * 4
	return zlib.crc32(data[:dataSizeMod4], 0)

def grmParseState(version, numDoF, dataNoHeader):
	state = {}
	if version == 1:
		print state
		#, euler, joint_pos, userData, currIndex-headerSizerosPubDec
		state1 = struct.unpack(state1Fmt, dataNoHeader[:state1Size])
		state['millis'] = state1[0]
		state['cmdMode'] = state1[1]
		state['behaviorId'] = state1[2]
		state['behaviorMode'] = state1[3]

		state['imu/linear_acceleration'] = np.array(state1[4:7])
		state['imu/angular_velocity'] = np.array(state1[7:10])
		state['imu/euler'] = np.array(state1[10:13])
		state['imu/orientation_covariance'] = np.array(state1[13:22])
		state['imu/linear_acceleration_covariance'] = state1[22]
		state['imu/angular_velocity_covariance'] = state1[23]

		state['battery/voltage'] = state1[24]
		state['battery/current'] = state1[25]

		state['twist/linear'] = np.array(state1[26:29])
		state['twist/angular'] = np.array(state1[29:32])

		state['joy/axes'] = np.array(state1[32:36])
		state['joy/buttons'] = np.array(state1[36:46])

		# joints
		# The following is based on SDK.cpp. FIXME use the C program instead of the python one
		if robotType == RobotType.MINITAUR:
			ptol = [6, 7, 5, 4, 1, 0, 2, 3]
			# indexed by j
			zeros = [0.0, np.pi, np.pi, 0.0, 0.0, np.pi, np.pi, 0.0]
			dirs = [1, 1, 1, 1, -1, -1, -1, -1]
		else:
			ptol = [8, 0, 1, 10, 4, 5, 9, 2, 3, 11, 6, 7]
			# indexed by j
			zeros = [0.0, 0.5 * np.pi, 0.0, 0.5 * np.pi, 0.0, 1.5 * np.pi, 0.0, 1.5 * np.pi, 0.0, 0.0, 0.0, 0.0]
			dirs = [1, -1, 1, -1, -1, 1, -1, 1, -1, 1, -1, 1]
		ndof = len(ptol)
		state['joint/position'] = np.zeros((ndof))
		state['joint/velocity'] = np.zeros((ndof))
		state['joint/current'] = np.zeros((ndof))
		state['joint/temperature'] = np.zeros((ndof))
		state['joint/voltage'] = np.zeros((ndof))
		state['joint/effort'] = np.zeros((ndof))
		currIndex = state1Size

		for p in range(numDoF):
			pos, vel, curr, temp, volt, eff = struct.unpack(LANIFmt, dataNoHeader[currIndex:currIndex + LANISize])
			# p = physical address
			# j is the logical ID
			if p < ndof:
				j = ptol[p]
				jang = pos - zeros[j]
				# want between -pi,pi
				jang = (2 * np.pi + jang) % (2 * np.pi)
				if jang > np.pi:
					jang = jang - 2 * np.pi
				jang = jang * dirs[j]
				state['joint/position'][j] = jang
				state['joint/velocity'][j] = vel * dirs[j]
				# The following may not be available
				state['joint/current'][j] = 0.001 * curr
				state['joint/temperature'][j] = 0.01 * temp
				state['joint/voltage'][j] = 0.001 * volt
				state['joint/effort'][j] = eff
			currIndex += LANISize

		# user data
		state['userData'] = struct.unpack(userDataFmt, dataNoHeader[currIndex: currIndex + userDataSize])
		# currIndex += userDataSize

	return state

def openRXSocket():
	# Open RX socket
	srx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
	try:
		srx.bind(OBC_IP,OBC_PORT)
		print 'RX bind to port succeeded.', srx.getsockname()
	except socket.error as msg:
		print 'RX bind to port failed: ' + msg[1]
	return srx

def openTXSocket(iface):
	# Open TX socket
	stx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

	# Enable broadcast
	try:
		stx.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
	except socket.error as msg:
		print 'TX set broadcast failed: ' + msg[1]

	# Bind to network interface
	try:
		if not hasattr(socket, 'SO_BINDTODEVICE'):
			socket.SO_BINDTODEVICE = 25
		stx.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, iface+'\0')
		print 'TX bind to ' + iface + ' succeeded.'
	except socket.error as msg:
		print 'TX bind to ' + iface + ' failed: ' + msg[1]
	return stx

def transmitPacket(stx, packet):
	# Calculate CRC, pack in header, magic, and CRC, transmit
	dataCrc = grmCrc(packet)
	header = struct.pack(headerFmt, GRM_MAGIC, 1, len(packet), dataCrc, 16, 1)
	stx.sendto(header + packet, ('<broadcast>', MCU_PORT))

def receivePacket(srx):
	# print("receive")
	# RX. Keep receiving until it throws an exception, and take the freshest packet received.
	data = None
	addr = [0, 0]
	while True:
		# print("try to receive something")
		try:
			# sleep(1)
			data_received = srx.recv(4096)
		except socket.error, e:
			err = e.args[0]
			print "RX error "
			print os.strerror(err)


		data = data_received
		print data
		# addr = addr_received

	if addr[1] != MCU_SRC_PORT:
		return None, None

	try:
		magic, version, totalSize, headerCrc, numDoF, configBits = struct.unpack(headerFmt, data[:headerSize])
		# print("yes")

		if magic != GRM_MAGIC:
			print('Magic byte did not match')
			return None, None

		# Check crc: it is contained in the header, and a crc32 of all the bytes after the header
		if headerCrc != grmCrc(data[headerSize: headerSize + totalSize]):
			print('CRC did not match')
			return None, None

		state = grmParseState(version, numDoF, data[headerSize:headerSize+totalSize])
	except:
		return None, None
	return state, numDoF
