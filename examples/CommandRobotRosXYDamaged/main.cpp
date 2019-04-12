/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Tom Jacobs <tom.jacobs@ghostrobotics.io>, Avik De <avik@ghostrobotics.io>
 */
#include <Motor.h>
#include <ReorientableBehavior.h>
#include <SDK.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
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
struct SerialCommandPacket {
    uint32_t version;
    BehaviorCmd behavior_command;
    uint16_t checksum;
};

struct SerialCommandPacketXY {
    uint32_t version;
    float X[4];
    float Y[4];
    bool restart;
    uint16_t checksum;
};

const char ALIGNMENT_WORD[2] = {'G', 'R'};
//Struct of the packet to send
struct SerialStatePacket {
    char align[2];
    uint32_t millis, lastRX; // report last reception from the computer
    Vector3 euler; // some robot state
    float positions[8];
    float velocities[8];
    float torques[8];
    float currents[8];
    float temperatures[8];
    bool impossible_motion;
    uint16_t checksum;
};
#pragma pack(pop)

float angleCmd;
float extensionCmd;
float X[8] = {0.0, 0.0, 0.0, 0.0};
float Y[8] = {0.0, 0.0, 0.0, 0.0};
float R[8] = {0.14, 0.14, 0.14, 0.14};
float theta[8] = {0.0, 0.0, 0.0, 0.0};
bool restart = false;
float meanCurrent[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float temp[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

class CommandRobotXY : public ReorientableBehavior {
public:
    void begin()
    {
        angleCmd = 0;
        extensionCmd = 0.0;
        // Command by behavior
        C->mode = RobotCommand_Mode_LIMB;
    }

    // Parser state
    // Goes from 0 to 1 to 2
    int numAlignmentSeen = 0;
    uint16_t rxPtr = 0;

    // Receive buffer and alignment
    const static uint16_t RX_BUF_SIZE = 100;

    SerialCommandPacketXY commandPacket;
    SerialStatePacket statePacket;

    // Keep track of our last state packet send
    const static uint32_t TX_EVERY_MS = 100; //Can be diminished if the size of the state packet is lowered
    uint32_t lastTX = 0;
    uint32_t mean_counter = 0;
    uint32_t begin_meant = 0;

    //Safety check
    bool impossible_motion = false;
    // Helper function to calculate a simple sum-of-bytes checksum
    uint16_t bufChecksum(const uint8_t* buffer, uint16_t nbytes)
    {
        uint16_t sum = 0;
        for (uint16_t i = 0; i < nbytes; ++i) {
            sum += buffer[i];
        }
        return sum;
    }

    void update()
    {
        //Here we put a safety check, if the minitaur is trying to do a motion which requires to much torque/amps go to base position
        // float torque_sum = 0;
        // float current_sum = 0;

        // if ((torque_sum > 8) && (current_sum > 100)) { //8 and 100 have been determined by looking experimentaly at the values, with good and harsh motions
        //     impossible_motion = true;
        // }

        // Character to store the latest received character
        uint8_t latestRX;

        // Loop through while there are new bytes available
        while (read(STDIN_FILENO, &latestRX, 1) > 0) {
            if (numAlignmentSeen == 0 && latestRX == ALIGNMENT_WORD[0]) {
                numAlignmentSeen = 1;
            }
            else if (numAlignmentSeen == 1 && latestRX == ALIGNMENT_WORD[1]) {
                numAlignmentSeen = 2;
                rxPtr = 0;
            }
            else if (numAlignmentSeen == 2) {
                // Add the next byte to our memory space in serial_packet
                uint8_t* pSerialPacket = (uint8_t*)&commandPacket;
                pSerialPacket[rxPtr++] = latestRX; // post-increment rxPtr

                // Check if we have read a whole packet
                //printf("ptr %d %d\n", rxPtr, sizeof(SerialPacket));
                if (rxPtr == sizeof(SerialCommandPacketXY)) {
                    // Check checksum
                    uint16_t checksum = bufChecksum(pSerialPacket, sizeof(SerialCommandPacketXY) - 2);
                    if (commandPacket.checksum == checksum) {
                        // Check we're processing the right version
                        if (commandPacket.version == 1) {
                            //Recover x y coordinates and compute polar coordinates from it
                            memcpy(&X, &commandPacket.X, 4 * 4);
                            memcpy(&Y, &commandPacket.Y, 4 * 4);
                            for (unsigned int i = 0; i < 4; i++) {
                                R[i] = sqrt(pow(X[i], 2) + pow(Y[i], 2));
                                theta[i] = atan2(Y[i], X[i]);
                            }
                            R[0] = 0.14;
                            theta[0] = 0;
                            // Store last received time
                            statePacket.lastRX = S->millis;
                        }
                        if (commandPacket.restart == true) {
                            impossible_motion = false;
                            for (int i = 0; i < P->limbs_count; ++i) {
                                R[i] = 0.14;
                                theta[i] = 0;
                            }
                        }
                    }

                    // Reset
                    numAlignmentSeen = rxPtr = 0;
                }
            }
        }
        //Control the limbs thanks to the received cmds
        for (int i = 0; i < P->limbs_count; ++i) {
            P->limbs[i].type = LimbParams_Type_SYMM5BAR_EXT_M;
            // Splay angle for the front/rear legs (outward splay due to fore-displacement of front legs
            // and aft-displacement of rear legs)
            // The pitch angle (S->imu.euler.y) is subtracted since we want to the set the *absolute* leg angle
            // and limb[i].setPosition(ANGLE, *) will set the angle of the leg *relative* to the robot body

            // angDes = (isFront(i)) ? -S->imu.euler.y - 0.1 : -S->imu.euler.y + 0.2;
            if (impossible_motion == true) {
                limb[i].setGain(ANGLE, 0, 0);
                limb[i].setGain(EXTENSION, 0, 0);
            }
            else {
                if (i == 0) {
                    limb[i].setGain(ANGLE, 0, 0);
                    limb[i].setGain(EXTENSION, 0, 0);
                }
                else {
                    limb[i].setGain(ANGLE, 0.9, 0.03);
                    limb[i].setGain(EXTENSION, 120, 3);
                    limb[i].setPosition(ANGLE, theta[i]);
                    // Set the leg extension to 0.14 m
                    limb[i].setPosition(EXTENSION, R[i]);
                }
            }
        }

        if (mean_counter == 0) {
            begin_meant = S->millis;
        }
        for (unsigned int j = 0; j < S->joints_count; j++) {
            meanCurrent[j] += abs(S->joints[j].current);
        }
        mean_counter++;
        if ((S->millis - begin_meant) > 1000) {
            for (unsigned int j = 0; j < S->joints_count; j++) {
                temp[j] = meanCurrent[j] / mean_counter;
                if (meanCurrent[j] / mean_counter > 25) {
                    impossible_motion = true;
                }
                meanCurrent[j] = 0;
            }
            mean_counter = 0;
        }

        // Recover and send state
        if (S->millis - lastTX > TX_EVERY_MS) {
            lastTX = statePacket.millis = S->millis;
            memcpy(statePacket.align, ALIGNMENT_WORD, 2);
            memcpy(&statePacket.euler, &S->imu.euler, sizeof(Vector3));

            for (unsigned int j = 0; j < S->joints_count; j++) {
                statePacket.positions[j] = S->joints[j].position;
                statePacket.velocities[j] = S->joints[j].velocity;
                statePacket.torques[j] = S->joints[j].torqueEst;
                statePacket.currents[j] = S->joints[j].current;
                statePacket.temperatures[j] = S->joints[j].temperature;
                statePacket.temperatures[j] = temp[j];
            }
            // memcpy(&statePacket.theta, &theta, 8 * 4);
            // memcpy(&statePacket.R, &R, 8 * 4);
            statePacket.impossible_motion = impossible_motion;
            statePacket.checksum = bufChecksum((const uint8_t*)&statePacket, sizeof(SerialStatePacket) - 2);
            write(STDOUT_FILENO, &statePacket, sizeof(statePacket));
        }
    }
    // ::end() is called when the behavior is stopped
    void
    end()
    {
    }
};

void debug()
{

    // printf("angleCmd %f\t", angleCmd);
    // printf("extensionCmd %f\t", extensionCmd + 0.14);
    // printf("angDes %f\t", angDes);

    // for (unsigned int i = 0; i < 8; i++) {
    //     printf("X Y   %.2f\t  %.2f\t", X[i], Y[i]);
    //     printf("R theta   %.2f\t  %.2f\t", R[i], theta[i]);
    // }
}
int main(int argc, char* argv[])
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
    // setDebugRate(100);

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
    CommandRobotXY commandRobotXY;
    behaviors.clear();
    behaviors.push_back(&commandRobotXY);
    commandRobotXY.begin();

    // Go
    return begin();
}
