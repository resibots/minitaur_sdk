# Minitaur SDK extension

This repository is an extension of the official Ghost Robotics SDK.
Please refer to the installation instruction avaible here : http://ghostrobotics.gitlab.io/SDK/Installation.html

The difference with the official SDK is that this repository contains the following example :

* CommandRobotRosXY

CommandRobotRosXY allows to send cartesian end leg position commands to the minitaur, via the topic  */robot0/command/xy_cmd*

## How to use CommandRobotRosXY ?

Open a new terminal and type :
* roscore


Connect to the minitaur via USB
* cd path_to_CommandRobotRosXY/
* make
* sudo -s

If needed change the usb port adress in command.py ( default is "/dev/ttyUSB0" )

* python command.py

Now everything has been launched you should be able to use the following topics :

```
/robot0/command/xy_cmd
/robot0/state/currents
/robot0/state/impossible_motion
/robot0/state/imu
/robot0/state/joint
/robot0/state/jointURDF
/robot0/state/max
/robot0/state/pose
/robot0/state/sum
/robot0/state/temperatures
/rosout
/rosout_agg
```

## Topics details


``` /robot0/command/xy_cmd ```

 You can send end leg position commands to this topic in this order : x1,x2,x3,x4,y1,y2,y3,y4.
 With (x1,y1) the end leg position of the first leg in cartesian coordinates.


``` /robot0/command/currents ```

Display the motor ampere consumption


``` /robot0/state/imu ```

 Display the imu data

``` /robot0/state/joint ```

Display the joint information including joint positions and velocities

``` /robot0/state/jointURDF ```

Display the joint information in another format to be used with the minitaur_rviz.launch in thirdparty/minitaur_description/launch/minitaur_rviz.launched

```/robot0/state/temperatures ```

 Display the motor temperatures

```/robot0/state/pose ```

 For now same as  ``` /robot0/state/imu ```

```/robot0/state/max ```

 For debug only, maximum torque and current detected

```/robot0/state/sum ```

For debug only sum of torques and current

```/robot0/state/impossible_motion ```

Publish if the minitaur has detected an impossible motion or not.

When the minitaur tries to execute a motion that requires a motor to get more than 25A for more than 1second, the motion will be declared as impossible.

If an impossible motion has been detected, the minitaur will put the PD gains of the motors to 0. You will then need to call the rosservice ```/restart``` in order to disable the safety and start a new motion.
