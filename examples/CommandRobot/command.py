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

# Optional: Try to use pygame for joystick input
# Mac: brew install sdl sdl_image sdl_mixer sdl_ttf portmidi
# sudo pip install pygame
joystickFound = False
try:
    import pygame
    pygame.display.init()
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(x) for x in range(pygame.joystick.get_count())]

    # Get count of joysticks
    joystick_count = pygame.joystick.get_count()
    print("Number of joysticks: {}".format(joystick_count))

    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        print("Joystick {}".format(i) )

        # Get the name from the OS for the controller/joystick
        name = joystick.get_name()
        print("Joystick name: {}".format(name) )

        # Usually axis run in pairs, up/down for one, and left/right for the other.
        axes = joystick.get_numaxes()
        print("Number of axes: {}".format(axes) )

        # Buttons
        buttons = joystick.get_numbuttons()
        print("Number of buttons: {}".format(buttons) )

        # Hat switch. All or nothing for direction, not like joysticks.
        hats = joystick.get_numhats()
        print("Number of hats: {}".format(hats) )

        joystickFound = True

except:
    print("No pygame found, continuing.")

# Test by commanding forwards and backwards
phase = 0.0

def calculateChecksum(bytes, length):
    checksum = ct.c_ushort(0)
    for i in range(length):
        checksum = ct.c_ushort(int(checksum.value) + ord(bytes[i]));
    return checksum.value

while(1):

    if joystickFound:
        pygame.event.pump()
        for i in range( axes ):
            axis = joystick.get_axis( i )
            print("Axis {} value: {:>6.3f}".format(i, axis) )
        for i in range( buttons ):
            button = joystick.get_button( i )
            print("Button {:>2} value: {}".format(i, button) )
        for i in range( hats ):
            hat = joystick.get_hat( i )
            print("Hat {} value: {}".format(i, str(hat)) )

    # Init
    linear = {}
    angular = {}
    position = {}
    orientation = {}

    # Set values
    id = 0
    mode = 1                                  # BehaviorMode_RUN
    if not joystickFound:
        linear['x'] = 0.2 * math.sin(phase/20.0) # Walk forwards then backwards, over and over
        linear['y'] = 0
        linear['z'] = 0
        angular['x'] = 0
        angular['y'] = 0
        angular['z'] = 0
        position['x'] = 0
        position['y'] = 0
        position['z'] = 0
        orientation['x'] = 0
        orientation['y'] = 0
        orientation['z'] = 0
        orientation['w'] = 0
    else:
        linear['x'] = -joystick.get_axis(3) # Forwards and backwards
        linear['y'] = 0
        linear['z'] = 0
        angular['x'] = 0
        angular['y'] = 0
        angular['z'] = -joystick.get_axis(2) # Yaw
        position['x'] = 0
        position['y'] = 0
        position['z'] = -joystick.get_axis(1) # Height
        orientation['x'] = 0
        orientation['y'] = 0
        orientation['z'] = 0
        orientation['w'] = 0

    # print("Sending walking speed:  " + str(linear['x']))

    # Pack BehaviorCmd. Definition is at the end of this file in comments.
    # Packet version number, BehaviorCmd: {id, twist.linear, twist.angular, pose.position, pose.orientation, mode}
    behavior_command = struct.pack( '<II3f3f3f4fI', 1, # Version 1 of serial packet format
                                                    id,
                                                    linear['x'], linear['y'], linear['z'],
                                                    angular['x'], angular['y'], angular['z'],
                                                    position['x'], position['y'], position['z'],
                                                    orientation['x'], orientation['y'], orientation['z'], orientation['w'],
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
                    # Parse packet into tuple: time, lastRX time, 3 float robot state, checksum
                    tup = struct.unpack('<2I3fH', chunk)
                except:
                    continue

                # Get times
                time = tup[0]
                lastRXtime = tup[1]

                # Compare checksum
                checksum = tup[5]
                checksumCalculated = calculateChecksum('GR' + chunk, 2+4+4+(3*4))

                # Print state
                if(checksum == checksumCalculated):
                    print("Times: " + str(time) + " " + str(lastRXtime) + " " + str(time-lastRXtime) + " Robot IMU state: " + str(tup[2]) + " " + str(tup[3]) + " " + str(tup[4]))

        # Put back the last (possibly unfinished) chunk
        rxBuf = chunks[-1]

    # Increase phase
    phase += 1

    # Pause for 10ms
    sleep(0.01)

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
