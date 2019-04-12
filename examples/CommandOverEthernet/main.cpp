/*
MIT License (modified)

Copyright (c) 2018 Ghost Robotics
Author: Avik De <avik@ghostrobotics.io> and Tom Jacobs <tom.jacobs@ghostrobotics.io>

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
 */
#include <MAVCmd.h>
#include <Motor.h>
#include <ReorientableBehavior.h>
#include <SDK.h>
#include <stdio.h>
#include <unistd.h>
using namespace std;

#pragma pack(push, 1)
// Custom data to append to ethernet packet. No more than GRM_USER_DATA_SIZE=32 bytes!
struct UserData {
    uint8_t bytes[GRM_USER_DATA_SIZE];
};
#pragma pack(pop)

uint32_t ethRxUpdated;
bool prevJoyRCEnabled = false;
bool joyRCEnabled = true;
bool ether = false;
bool ether1 = false;
bool ether2 = false;
bool ether3 = false;
int id = -1;
int mode = -1;
float linearx = -1;
float linearz = -1;
float height = -1;
// int bytes = -1;

class Default : public Peripheral {
public:
    void begin()
    {
    }

    // Our received behaviorCmd and user data
    BehaviorCmd behaviorCmd;
    UserData data;

    void update()
    {
        // If no data, stop the robot
        if (S->millis > ethRxUpdated + 2000) // If it's too long past last time we received a packet
        {
            // Stop and re-enable the joystick
            if (!joyRCEnabled) {
                C->behavior.mode = 0;
                C->behavior.twist.linear.x = 0;
                joyRCEnabled = true;
            }
        }
        else {
            // Read BehaviorCmd from ethernet
            int numRead = read(ETH_UPSTREAM_FILENO, &behaviorCmd, sizeof(BehaviorCmd));
            int numRead = 0;
            if (numRead == sizeof(BehaviorCmd)) {
                printf("1\n");
                // If sensible command
                if (behaviorCmd.id < 10 && behaviorCmd.mode < 10 && behaviorCmd.twist.linear.x <= 1.0 && behaviorCmd.twist.linear.x >= -1.0 && behaviorCmd.twist.angular.z <= 1.0 && behaviorCmd.twist.angular.z >= -1.0) {
                    // Copy to behavior
                    printf("2\n");
                    memcpy(&C->behavior, &behaviorCmd, sizeof(BehaviorCmd));

                    // Disable RC joystick
                    joyRCEnabled = false;
                }
            }
        }

        // // Disable or enable joystick once
        // if (joyRCEnabled && !prevJoyRCEnabled) {
        //     // Enable joystick input
        //     JoyType joyType = JoyType_FRSKY_XSR;
        //     ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
        //     printf("Enabling RC Joystick\n");
        //     C->mode = RobotCommand_Mode_LIMB;
        // }
        // if (!joyRCEnabled && prevJoyRCEnabled) {
        //     // Disable joystick input
        //     JoyType joyType = JoyType_NONE;
        //     ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);
        //     printf("Disabling RC Joystick\n");
        //     C->mode = RobotCommand_Mode_BEHAVIOR;
        // }
        // prevJoyRCEnabled = joyRCEnabled;
        printf("3\n");
        for (int i = 0; i < GRM_USER_DATA_SIZE; ++i) {
            data.bytes[i] = i;
        }
        int bytes = write(ETH_UPSTREAM_FILENO, &data, sizeof(UserData));
        printf("bytes  %d\t", bytes);
        //Ceate example user bytes to send back to computer
    }
};

// You can optionally send back a fully custom state packet by replacing the state copy callback.
// If you do this, don't write user bytes using the example user bytes above.
uint16_t myStateCopyCallback(GRMHeader* hdr, uint8_t* buf)
{
    // Set your own version to keep track of parsing ability
    hdr->version = 1;
    int offset = 0;
    memcpy(buf, &S->millis, sizeof(S->millis));
    offset += sizeof(S->millis);

    // IMU
    memcpy(&buf[offset], &S->imu.angular_velocity, sizeof(Vector3));
    offset += sizeof(Vector3);
    memcpy(&buf[offset], &S->imu.euler, sizeof(Vector3));
    offset += sizeof(Vector3);

    // Need to return the size at the end
    return offset;
}

void debug()
{
    // int behavnb = 0;
    // for (auto& b : behaviors) {
    //     behavnb++;
    // }
    // if (ether)
    //     printf("Ether\t");
    //
    // if (ether1)
    //     printf("Ether1\t");
    //
    // if (ether2)
    //     printf("Ether2\t");
    //
    // if (ether3) {
    //     printf("Ether3\t");
    // printf("Behavior nb %d\t", behavnb);
    // printf("BehaviorCmd id %d\t", id);
    // printf("BehaviorCmd mode %d\t", mode);
    // printf("BehaviorCmd linear x  %.2f\t", linearx);
    // printf("BehaviorCmd linear z %.2f\t", linearz);
    // printf("BehaviorCmd height z %.2f\t", height);
    // }
    // printf("Behavior bytes %d\t", bytes);

    // Show joint positions
    // for (uint8_t i = 0; i < P->joints_count; ++i)
    //     printf("%.2f\t", joint[i].getPosition());

    // Show raw toe values
    // uint16_t rawToe[4];
    // ioctl(TOE_SENSORS_FILENO, IOCTL_CMD_RD, rawToe);
    // for (uint8_t i = 0; i < P->limbs_count; ++i)
    //     printf("%d\t", rawToe[i]);

    // Show IMU info
    // printf("%.2f\t%.2f\t%.2f\t", S->imu.euler.x, S->imu.euler.y, S->imu.euler.z);
    // printf("%.2f\t%.2f\t%.2f\t", S->imu.angular_velocity.x, S->imu.angular_velocity.y, S->imu.angular_velocity.z);
    // printf("%.2f\t%.2f\t%.2f\t", S->imu.linear_acceleration.x, S->imu.linear_acceleration.y, S->imu.linear_acceleration.z);

    // printf("\n");

    // Update the ethernet and get the last update time
    // ioctl(ETH_UPSTREAM_FILENO, IOCTL_CMD_GET_LAST_UPDATE_TIME, &ethRxUpdated);
}

int main(int argc, char* argv[])
{
#if defined(ROBOT_MINITAUR)
    init(RobotParams_Type_MINITAUR, argc, argv);
    for (int i = 0; i < P->joints_count; ++i)
        P->joints[i].zero = motZeros[i];
#elif defined(ROBOT_MINITAUR_E)
    init(RobotParams_Type_MINITAUR_E, argc, argv);
    // setDebugRate(100);
#else
#error "Define robot type in preprocessor"
#endif

    // Enable remote control
    JoyType joyType = JoyType_FRSKY_XSR;
    ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);

    // Create controller peripheral
    Default commandRobot;
    commandRobot.begin();

    // Add it
    addPeripheral(&commandRobot);

    // Remove bound behavior from Minitaur (first element of behaviors vector),
    // so we're left with only walk behavior
    behaviors.erase(behaviors.begin());

    //     //  If physical robot
    // #if defined(ARM_MATH_CM4)
    //     bool ethernet_robot_control = true;
    //     if (ethernet_robot_control) {
    //         // Enable Robot Control SDK (via ethernet)
    //         mavCmd.begin();
    //         addPeripheral(&mavCmd);
    //     }
    //     else {
    //         // Replace state copy callback to send back custom logging info (via ethernet)
    //         ioctl(ETH_UPSTREAM_FILENO, IOCTL_CMD_STATE_COPY_CALLBACK, (void*)myStateCopyCallback);
    //     }
    // #endif

    return begin();
}
