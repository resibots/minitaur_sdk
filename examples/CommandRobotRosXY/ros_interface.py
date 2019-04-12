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

See command.py

'''

''' ROS interface for usb example '''
bROS = False
try:
	import rospy
	import numpy as np
	import tf
	import std_msgs
	from sensor_msgs.msg import Imu, BatteryState, JointState, Joy
	from std_msgs.msg import UInt32, Float64MultiArray, Bool
	from nav_msgs.msg import Odometry
	from geometry_msgs.msg import Twist, Pose
	from std_srvs.srv import Empty, EmptyResponse
	from trajectory_msgs.msg import JointTrajectory
	bROS = True
except:
	print('No ROS found; continuing.')

def toContinue():
	if bROS:
		return not rospy.is_shutdown()
	else:
		return True

# ROS variables
robotname = "robot0"
publish_time = 0
pubs = []
subs = []

# Commands received from ROS
behaviorId = 0
behaviorMode = 0
height = 0
lateral = 0
count = 0
linear = [0,0,0]
angular = [0,0,0]
position = [0,0,0]
orientation = [0,0,0,0]
X=[0.14,0.14,0.14,0.14]
Y=[0,0,0,0]
x_low_lim = 0.09
x_high_lim = 0.3
y_low_lim = -0.10
y_high_lim = 0.10
impossible_motion = True
restart = False
def initROS():
	global pubs, subs, robotname

	# Get robot name param from ROS
	if bROS:
		try:
			if rospy.has_param('robotname'):
				robotname = rospy.get_param('robotname')
		except:
			print("ROS master not running.")

		# Create publishers and subscribers
		pubs = [
			rospy.Publisher(robotname + '/state/imu', Imu, queue_size=10),
			rospy.Publisher(robotname + '/state/joint', JointState, queue_size=10),
			rospy.Publisher(robotname +'/state/jointURDF', JointState, queue_size=10),
			rospy.Publisher(robotname + '/state/pose', Pose, queue_size=10),
			rospy.Publisher(robotname + '/state/currents', Float64MultiArray, queue_size=10),
			rospy.Publisher(robotname + '/state/mean_currents', Float64MultiArray, queue_size=10),
			rospy.Publisher(robotname + '/state/max', Float64MultiArray, queue_size=10),
			rospy.Publisher(robotname + '/state/sum', Float64MultiArray, queue_size=10),
			rospy.Publisher(robotname + '/state/impossible_motion', Bool, queue_size=10),
		]
		subs = [
			rospy.Subscriber(robotname + '/command/xy_cmd',Float64MultiArray, xy_cmd_callback),
		]

		# Init ROS node
		rospy.init_node('ethernet_robot_control')

		def restart(req):
			global restart
			restart = True
			return EmptyResponse()
		rospy.Service('restart', Empty, restart)


def getCommands():
	global X ,Y ,linear, angular, position, orientation, behaviorId, behaviorMode, height, lateral, restart
	re = False
	if(restart == True):
		restart = False
		re = True
	return X ,Y ,linear, angular, position, orientation, behaviorId, behaviorMode, height, lateral, re

def joy_callback(data):
	global height, lateral
	height = mapFromTo(data.axes[2], -1.0, 1.0, -1.0, 0.6)
	lateral = mapFromTo(data.axes[3], -1.0, 1.0, -1.0, 1.0)
	#rospy.loginfo("Received: %.2f Set: %.2f", data.axes[2], height)

def cmd_vel_callback(data):
	global linear, angular
	#rospy.loginfo("Received:\n %s", data)
	linear[0] = data.linear.x if abs(data.linear.x)<0.4 else 0.4*np.sign(data.linear.x)
	linear[1] = data.linear.y
	linear[2] = data.linear.z
	angular[0] = data.angular.x
	angular[1] = data.angular.y
	angular[2] = data.angular.z if abs(data.angular.z)<0.9 else 0.9*np.sign(data.angular.z)


def cmd_pose_callback(data):
	global position, orientation
	#rospy.loginfo("Received:\n %s", data)
	position[0] = data.position.x
	position[1] = data.position.y
	position[2] = data.position.z if abs(data.position.z)<1.4 else 1.4*np.sign(data.position.z)
	orientation[0] = data.orientation.x
	orientation[1] = data.orientation.y
	orientation[2] = data.orientation.z
	orientation[3] = data.orientation.w


def xy_cmd_callback(data):
	global x_high_lim, x_low_lim, y_high_lim, y_low_lim, X,Y
	j = 0
	if(len(data.data)!=8):
		"Warning not enough X, Y coordinates. Please send 8 joint trajectories (X and Y) in this order : x1,x2,x3,x4,y1,y2,y3,y4"
		X=[0.14,0.14,0.14,0.14]
		Y=[0,0,0,0]
	else:
		for i in range(0,len(data.data)):
			position = data.data[i]
			if(i<4):
				if(position > x_high_lim):
					position = x_high_lim
				if(position < x_low_lim):
					position = x_low_lim
				X[i] = position
			if(i>=4):
				if(position > y_high_lim):
					position = y_high_lim
				if(position < y_low_lim):
					position = y_low_lim
				Y[j] = position
				j=j+1

def behaviorId_callback(data):
	global behaviorId
	#rospy.loginfo("Received:\n %s", data.data)
	behaviorId = data.data

def behaviorMode_callback(data):
	global behaviorMode
	#rospy.loginfo("Received:\n %s", data.data)
	behaviorMode = data.data

def minitaurFKForURDF(t0,t1):
	l1 = 0.1
	l2 = 0.2
	meanAng = 0.5 * (t0 + t1)
	diffAng = 0.5 * (t0 - t1)
	if (meanAng < 0):
		meanAng = meanAng + np.pi
		diffAng = diffAng + np.pi
	l1c = l1 * np.cos(meanAng)
	l1s = l1 * np.sin(meanAng)
	r = np.sqrt(l2 * l2 - l1s * l1s) - l1c
	#print r
	if r < 0:
		r = -r
	# r tested - looks right
	# stupid law of cosines
	phi = np.arccos((l2 * l2 + l1 * l1 - r * r)/ (2 * l1 * l2))
	return np.pi - phi, np.pi + phi, r

# def minitaurFKForURDF(t0,t1):
# 	l1 = 0.1
# 	l2 = 0.2
# 	meanAng = 0.5 * (t0 + t1) + np.pi
# 	diffAng = 0.5 * (t0 - t1)
# 	# if (diffAng < 0):
# 	# 	meanAng = meanAng + np.pi
# 	# 	diffAng = diffAng + np.pi
# 	l1c = l1 * np.cos(diffAng)
# 	l1s = l1 * np.sin(diffAng)
# 	r = np.sqrt(l2 * l2 - l1s * l1s) - l1c
# 	#print r
# 	if r < 0:
# 		r = -r
# 	# r tested - looks right
# 	# stupid law of cosines
# 	phi = np.arccos((l2 * l2 + l1 * l1 - r * r)/ (2 * l1 * l2))
# 	return np.pi - phi, np.pi + phi, r




def publishStateRos(tup, ros_pub_dec):
	# Publish our robot state to ROS topics /robotname/state/* periodically
	global publish_time, count, impossible_motion
	publish_time += 1
	# print(tup)
	if bROS:
		publish_time = 0
		# Construct /robotname/state/imu ROS message
		msg = Imu()
		roll = tup[2]
		pitch = tup[3]
		yaw = tup[4]
		positions = tup[5:13]
		#print positions
		velocities = tup[13:21]
		#print velocities
		torques = tup[21:29]
		# #print torques
		currents = tup[29:37]
		# print("Torque : ", max(np.array(torques)), min(np.array(torques)))
		# print("Currents : ", max(np.array(currents)), min(np.array(currents)))
		temperatures = tup[37:45]
		impossible_motion = tup[45]
		quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
		msg.orientation.x = quaternion[0]
		msg.orientation.y = quaternion[1]
		msg.orientation.z = quaternion[2]
		msg.orientation.w = quaternion[3]
		pubs[0].publish(msg)
		# Construct /robotname/state/pose
		msg = Pose()
		msg.orientation.x = quaternion[0]
		msg.orientation.y = quaternion[1]
		msg.orientation.z = quaternion[2]
		msg.orientation.w = quaternion[3]
		# TODO: Get height from robot state, have robot calculate it
		msg.position.z = 0.0
		pubs[3].publish(msg)

		msg = JointState()
		msg.name = []
		msg.position = []
		msg.velocity = []
		msg.effort = []
		for j in range(8):
			msg.name.append(str(j))
			msg.position.append(positions[j])
			msg.velocity.append(velocities[j])
			msg.effort.append(torques[j])
		pubs[1].publish(msg)

	    # Translate for URDF in NGR
		vision60 = False
		if(vision60):
			for i in range(8, 2):
				msg.position[i] += msg.position[i-1];
				msg.velocity[i] += msg.velocity[i-1];
		else:
			# other URDF
			# for URDF of Minitaur FIXME use the class I put in ethernet.py for RobotType
			msg.header.seq = count
			count = count +1
			msg.header.stamp =  rospy.Time.now()
			msg.position.extend([0, 0, 0, 0, 0, 0, 0, 0])
			msg.velocity.extend([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
			msg.effort.extend([0, 0, 0, 0, 0, 0, 0, 0])
			msg.name.extend(['8', '9', '10', '11', '12', '13', '14', '15'])
			msg.position[11], msg.position[10], r = minitaurFKForURDF(msg.position[0], msg.position[1])
			msg.position[14], msg.position[15], r = minitaurFKForURDF(msg.position[2], msg.position[3])
			msg.position[9], msg.position[8], r = minitaurFKForURDF(msg.position[4], msg.position[5])
			msg.position[13], msg.position[12], r = minitaurFKForURDF(msg.position[6], msg.position[7])
			# other URDF problems (order important)
			msg.position[11] = -msg.position[11]
			msg.position[14] = -msg.position[14]
			msg.position[9] = -msg.position[9]
			msg.position[13] = -msg.position[13]
		pubs[2].publish(msg)
		msg = Float64MultiArray()
		msg.data = currents
		pubs[4].publish(msg)
		msg = Float64MultiArray()
		msg.data = temperatures
		pubs[5].publish(msg)
		msg = Float64MultiArray()
		msg.data = [max(abs(np.array(torques))), max(abs(np.array(currents)))]
		pubs[6].publish(msg)
		msg = Float64MultiArray()
		msg.data = [sum(abs(np.array(torques))), sum(abs(np.array(currents)))]
		pubs[7].publish(msg)
		msg = Bool()
		msg.data = impossible_motion
		pubs[8].publish(msg)
