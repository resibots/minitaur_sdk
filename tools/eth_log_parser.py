'''
MIT License (modified)

Copyright (c) 2018 Ghost Robotics
Authors:
Avik De <avik@ghostrobotics.io>

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

'''
Documentation:
- log data using a custom ethernet state copy callback
- depending on your computer's ethernet driver or switch latency, you might want to log S->millis
- open wireshark to log data (connect ethernet cable to mbm bottom port)
- install `pip install scapy`
- provide log filename as argument
'''

import scapy.all
import struct
import sys
# for data manipulation
import numpy as np
import matplotlib.pyplot as plt

# Header
headerFmt = '<BBHiBB'
headerSize = struct.calcsize(headerFmt)

t = np.zeros((0))

# # Custom data: millis + 2x Imu
# ImuFmt = '9f'
# dataFmt = '<I' + ImuFmt + ImuFmt
# euler1 = np.zeros((0, 3))
# angular_velocity1 = np.zeros((0, 3))
# linear_acceleration1 = np.zeros((0, 3))
# euler2 = np.zeros((0, 3))
# angular_velocity2 = np.zeros((0, 3))
# linear_acceleration2 = np.zeros((0, 3))

# Custom data: millis + Njoints * (raw joint data)
dataFmt = '<I2f'
t = np.zeros((0))
cur = np.zeros((0))
curDes = np.zeros((0))

dataSize = struct.calcsize(dataFmt)
# rdpcap comes from scapy and loads in our pcap file
packets = scapy.all.rdpcap(sys.argv[1])

# Let's iterate through every packet
for packet in packets:
	if packet.haslayer(scapy.all.IP):
		if packet.getlayer(scapy.all.IP).src != '169.254.98.123':
			continue
			
	if packet.haslayer(scapy.all.UDP):
		if packet.getlayer(scapy.all.UDP).dport != 15000:
			continue
		alldata = packet.getlayer(scapy.all.UDP).payload.load
		magic, version, totalSize, headerCrc, numDoF, configBits = struct.unpack(headerFmt, alldata[:headerSize])
		# can check version here
		data = struct.unpack(dataFmt, alldata[headerSize: headerSize + dataSize])

		offset = 0
		t = np.hstack((t, data[offset]))
		offset += 1
		cur = np.hstack((cur, data[offset]))
		offset += 1
		curDes = np.hstack((curDes, data[offset]))
		offset += 1
		
print 'Average data rate =', 1.0/np.mean(np.diff(t)), 'Hz'

plt.figure()

# titles = ['Roll', 'Pitch', 'Yaw']
# for i in range(3):
# 	plt.subplot(3,1,i+1)
# 	plt.plot(t, np.unwrap(euler1[:, i]))
# 	plt.plot(t, np.unwrap(euler2[:, i]))
# 	plt.title(titles[i])
# plt.figure()
# titles = ['wx', 'wy', 'wz']
# for i in range(3):
# 	plt.subplot(3, 1, i+1)
# 	plt.plot(t, angular_velocity1[:, i])
# 	plt.plot(t, angular_velocity2[:, i])
# 	plt.title(titles[i])
# plt.figure()
# titles = ['ax', 'ay', 'az']
# for i in range(3):
# 	plt.subplot(3, 1, i+1)
# 	plt.plot(t, linear_acceleration1[:, i])
# 	plt.plot(t, linear_acceleration2[:, i])
# 	plt.title(titles[i])
		
# joints
plt.plot(t, cur)
plt.plot(t, curDes)


plt.show()
