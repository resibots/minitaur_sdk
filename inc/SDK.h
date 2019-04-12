/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef SDK_h
#define SDK_h

#include "simple.pb.h"
#include "DebugOutput.h"
#include "SMath.h"
#include "Peripheral.h"
#include "Behavior.h"

#include <vector>
#include <string>

#if defined(_MSC_VER)
typedef int64_t useconds_t;
#define UNUSED_PARAM
// unistd
#define STDIN_FILENO 0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2
// These are POSIX
#define write _write
#define read _read
extern "C" int _write(int file, char *data, int len);
extern "C" int _read(int file, char *ptr, int len);
#else
#include <sys/types.h> // for useconds_t
#include <unistd.h>
// #define UNUSED_PARAM __attribute__((unused_))
#endif

/** @addtogroup SDK Basic SDK operations
 *  @{
 */

/**
 * Defined separately for each platform, in units of Hz
 */
extern int CONTROL_RATE;

/**
 * Pointer to the global RobotState
 */
extern RobotState *S;
/**
 * Pointer to the global RobotCommand
 */
extern RobotCommand *C;
/**
 * Pointer to the global RobotParams
 */
extern RobotParams *P;

// for _write, special FILE*
//#define LOGGER_FILENO								3

/**
 * @brief Value of BehaviorCmd::mode that should stop the behavior
 */
#define BehaviorMode_STOP		(0)
/**
 * @brief Value of BehaviorCmd::mode that should start the behavior. Higher values can be customized for each behavior
 */
#define BehaviorMode_RUN		(1)

/**
 * @brief Globally defined vector of behavior (pointer)s that the user can append their own behaviors to
 * @details By default, this comes prepopulated with some number of behaviors (e.g. bound and walk for Minitaur; subject to grow)
 */
extern std::vector< Behavior *> behaviors;

/**
 * @brief System clock time (in microseconds)
 */
uint32_t clockTimeUS();
/**
 * @brief Low-level control time (in microseconds)
 */
extern uint32_t llControlTime;
/**
 * @brief Set on init
 */
extern bool jointCurrentControlAvailable;
/**
 * @brief Behavior update rate (Hz)
 */
extern uint32_t behaviorRate;
/**
 * @brief Behavior update time (microseconds)
 */
extern uint32_t behaviorTime;
/**
 * @brief Set to true on NGR2.5+ with hip-knee encoders
 */
extern bool bHipKneeEncoder;
extern bool bAbExtraFilt;
extern bool bEncExtraFilt;

/**
 * @brief Does the Minitaur have smaller legs than normal?
 */
extern bool bLittleLegs;

/**
 * @brief Hip/Knee velocity filter parameter
 */
extern float aHipKneeFilt;

/**
 * @brief Abduction velcoity filter parameter
 */
extern float aAbFilt;

/**
 * @brief State velocity filter parameter for X,Z
 */
extern float kLegKinR;

/**
 * @brief State velocity filter parameter for Y
 */
extern float kLegKinRy;

/**
 * @brief State acceleration filter
 */
extern float kImuR;

/**
 * @brief Initialize the SDK for a particular robot platform
 * 
 * @param type Robot type (see RobotParams::Type)
 * @param argc Number of command line arguments (pass from main())
 * @param argv Command line arguments (pass from main())
 * @return True if succeeded
 */
bool init(RobotParams_Type type, int argc, char *argv[]);

/**
 * @brief Commences the various control loops and tasks.
 * @details This should be the *last* function the user calls at the end of main. It never returns.
 * @return Never returns
 */
int begin();

/**
 * @brief Add to list of peripherals
 */
void addPeripheral(Peripheral *);

/**
 * @brief Enable or disable the safety shutoff heuristic. Leave on for a robot; can turn off for testing individual joints
 * @param flag True to enable
 */
void safetyShutoffEnable(bool flag);

/**
 * @brief Enable or disable the soft start init procedure
 * @details This behavior initializes the robot limbs in a way that avoid self-intersection and positions 
 * them for nominal operation when the robot first turns on
 * 
 * @param flag True to enable
 */
void softStartEnable(bool flag);

bool isSoftStartEnabled();

void *obc();

void *sim();

/**
 * @brief Microsecond sleep function. 
 * @details **Warning:** This kind of delay *must not* be used in Behavior::update(). This function
 * works the same in Unix-like systems and the MCU.
 * @param us Delay duration in microseconds
 */
extern "C" int usleep(useconds_t us);

/**
 * @brief Actuation update (critical, hard real-time)
 * @details Should be called by the implementation
 */
void sdkUpdateA(void *pvParameters);
/**
 * @brief Behavior and other task update (less critical, but still should be called as often as possition)
 * @details Should be called by the implementation
 */
void sdkUpdateB(void *pvParameters);

/** @} */ // end of addtogroup


/** @addtogroup IO Low-level input/output with file descriptors
 *  @{
 */

/**
 * Enum to store file descriptors on the MCU (for use with ioctl, write, read, ...).
 * unistd.h defines STDIN=0, STDOUT=1, STDERR=2. See https://en.wikipedia.org/wiki/File_descriptor
 * On the MCU all three of these point to the USB programming port
 */
enum MCUFD
{
	/**
	 * Control the user LED lighting.
	 * 
	 * Legacy usage (args == NULL, might be deprecated): command > 0 turns on the mainboard user LED, command = 0 turns it off
	 * 
	 * If args != NULL, command must be IOCTL_CMD_WR (for now). 
	 * 
	 * args is: uint8_t args[2] = {LED_ID, brightness}.
	 * 
	 * For digital LEDs, brightness=0 means off, brightness>0 means on. For PWM controlled LEDs, brightness is on a 0~255 scale.
	 * 
	 * The LED_ID is subject to change (see MCUClass.cpp or other platform-specific implementation), but for now: 0~9 = on-mainboard LEDs, 10~19 = OBC LEDs, 20~49 = limb-attached LEDs, 50~59 = body-attached LEDs, 60+ may be payload
	 */
	LED_USER_FILENO = 3,
	/**
	 * Control the 12V/1 power rail on the mainboard if present (command > 0 turns it on)
	 */
	PWR_12V1_FILENO,
	/**
	 * Control the 24V/1 power rail on the mainboard if present (command > 0 turns it on)
	 */
	PWR_24V1_FILENO,
	/**
	 * Control the V+/1 power rail on the mainboard if present (command > 0 turns it on)
	 */
	PWR_VIN1_FILENO,
	/**
	 * Control the Serial2 port (refer to MCU-specific information in the documentation)
	 */
	SERIAL_AUX_FILENO,
	/**
	 * Control the user SPI port (refer to MCU-specific information in the documentation)
	 */
	SPI_USER_FILENO,
	/**
	 * Control the user I2C port (refer to MCU-specific information in the documentation)
	 */
	I2C_USER_FILENO,
	/**
	 * Control the logger (refer to MCU-specific information in the documentation). Use write() to
	 * set some user-specified data for logging. As an example, if you have attached a user
	 * peripheral (such as an added sensor), you can append its readings to the logged data.
	 * The maximum size of this data is 32 bytes at the moment.
	 *
	 * Use ioctl() to control whether the onboard microSD card (when running on the MCU) is
	 * logging data (1 starts, 0 stops).
	 */
	LOGGER_FILENO,
	/**
	 * Control joystick sensitivities. 
	 * 
	 * Command = IOCTL_CMD_JOYSTICK_SET_SENS, in which case args is: float args[2] = {speed_sens, yaw_sens}
	 * 
	 * or command = IOCTL_CMD_JOYSTICK_SET_TYPE, in which case args is args = &joyType, where joyType is of type JoyType
	 */
	JOYSTICK_FILENO,
	/**
	 * Read in analog mode from GPIO pins. Command must be IOCTL_CMD_RD, and args is a {uint16_t=pin, uint16_t=result} tuple.
	 * 
	 * For example:
	 *    uint16_t params[2] = {pinNumber, 0};
	 *    ioctl(ADC_FILENO, IOCTL_CMD_RD, params);
	 *    uint16_t result = params[1];
	 */
	ADC_FILENO,
	/**
	 * Control toe-attached sensors. When using IOCTL_CMD_WR, passing a value of IOCTL_TOE_SENSOR_WR_IGNORE_MASK skips that toe.
	 */
	TOE_SENSORS_FILENO,
	/**
	 * Control digital pins. Command must be IOCTL_CMD_RD or IOCTL_CMD_WR, and args is a {uint16_t=pin, uint16_t=value/result} tuple.
	 * When using IOCTL_CMD_WR, the second element is the value written to the pin (0 or 1), and when using IOCTL_CMD_RD, result is updated with
	 * the logical value at the pin (0 or 1)
	 */
	DIO_FILENO,
	/**
	 * Upstream ethernet port (only on mbm). This is physically the lower RJ45 socket on mbm0.x.
	 * 
	 * read() can be called: only packets that properly match the header format will appear (see CommandOverEthernet example)
	 * The user cannot access this file in relay mode.
	 * 
	 * write() can be called to write to the userData region of the state packet a maximum of GRM_USER_DATA_SIZE bytes (see CommandOverEthernet example)
	 */
	ETH_UPSTREAM_FILENO,
};

// Coded colors for toe lighting (subject to change)
// Here is a uint8_t coding (bitwise) RRGGBBUU
// This will be transmitted over PWM and the last few bits are the worst affected
#define LED_CODE0_COLOR_MAGENTA (0b100010 << 2)
#define LED_CODE0_COLOR_CYAN (0b001010 << 2)
#define LED_CODE0_COLOR_YELLOW (0b101000 << 2)
#define LED_CODE0_COLOR_WHITE (0b101010 << 2)
#define LED_CODE0_COLOR_GRAY (0b010101 << 2)
#define LED_CODE0_COLOR_OFF (0)

/**
 * @brief MCU hardware device control function
 * @details This is meant to emulate userspace ioctl on Unix devices. For more information,
 * see https://www.gnu.org/software/libc/manual/html_node/IOCTLs.html or http://www.makelinux.net/ldd3/chp-6-sect-1.shtml
 * 
 * @param filedes An element of enum MCUFD, or STDIN_FILENO, STDOUT_FILENO, STDERR_FILENO
 * @param command Command to send device (see MCUFD for device-specific effect)
 * @param args Optional argument to command (usually a struct)
 *
 */
extern "C" int ioctl(int filedes, int command, void *args = NULL);

/**
 * For some ioctl() files (see MCUFD) such as I2C, the cmd can be to read or write. Use
 * cmd = IOCTL_CMD_RD to read.
 */
#define IOCTL_CMD_RD	(0)
/**
 * For some ioctl() files (see MCUFD) such as I2C, the cmd can be to read or write. Use
 * cmd = IOCTL_CMD_WR to write.
 */
#define IOCTL_CMD_WR (1)
#define IOCTL_CMD_WR_SCALE (2)
#define IOCTL_CMD_JOYSTICK_SET_SENS	(0)
#define IOCTL_CMD_JOYSTICK_SET_TYPE	(1)
#define IOCTL_CMD_ADC_READ IOCTL_CMD_RD
#define IOCTL_CMD_ADC_CTS_READ (2)
#define IOCTL_CMD_GET_LAST_UPDATE_TIME (2)
#define IOCTL_CMD_STATE_COPY_CALLBACK (3)

#define IOCTL_TOE_SENSOR_WR_IGNORE_MASK 0xffff

// Serial port configurations
#if !defined(SERIAL_8N1)
#define SERIAL_8N1 0x06
#define SERIAL_8N2 0x0E
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E
#endif

#if !defined(GRM_USER_DATA_SIZE)
#define GRM_USER_DATA_SIZE 32
#endif

// Single byte
#define GRM_MAGIC 0xab
// For compatibility with Microsoft Windows compilers, GCC supports a set of #pragma directives which change the maximum alignment of members of structures
#pragma pack(push, 1)
// Moved this one here from Messaging
typedef struct GRMHeader
{
	uint8_t magic, version; //magic MUST be GRM_MAGIC, otherwise nothing else is done
	uint16_t totalSize;			//size is the total packet size
	uint32_t checksum;			// EXCLUDES the header!
	uint8_t numDoF;
	uint8_t configBits; // lowest bit is 0 for "state" message, 1 for "command" message
} GRMHeader;
#pragma pack(pop)
typedef uint16_t (*StateCopyCallbackType)(GRMHeader *, uint8_t *);

#pragma pack(push,1)
typedef struct SerialPortConfig
{
	uint32_t baud;
	uint8_t mode;
} SerialPortConfig;
#pragma pack(pop)

#pragma pack(push, 1)
// <= 8-byte version for actual EEPROM writing
struct RobotInfo // goes in EEP_ROBOT_INFO_LOC
{
	uint8_t magic;
	uint8_t robotType;						// corresponds to P->type
	uint8_t robotVersion;					// e.g. 0x25 for NGR 2.5, 0x30 for v3 etc. Sets bHipKneeEncoder (ngr2.5)
	uint8_t encoderFiltConfig;		// bit 0 = bAbExtraFilt, bit 1 = bEncExtraFilt,
																// bits 2--3 Ab Filter parameter  0-.7, 1-.8, 2-.9, 3-.95
																// bits 4--5 hip and knee Filter parameter 0-0.7, 1-0.8, 2-.9, 3-.95
	uint8_t kinParamsConfig;			// bit 0 = bLittleLegs (for Minitaur)
	uint8_t stateEstParamsConfig; // bit 0--1 Acc filter 0:5, 1:8, 2:10, 3:13
																// bit 2--3 Vel Kin filter, X,Z 0:50, 1:100, 2:200, 3:300
																// bit 4--5 Vel Kin filter, Y 0:50, 1:100, 2:200, 3:300
																// bit 6--7 Momentum Observer, 0:50, 1:100, 2:200, 3:300 (Not yet implemented)
	uint8_t reserved[2];
};
#pragma pack(pop)
//  NV robot info
extern RobotInfo robotInfo;
/**
 * @brief args is interpreted as SerialPortConfig where mode is one of the above
 * 
 */
#define IOCTL_CMD_SERIAL_PORT_CFG (0)

/** @} */ // end of addtogroup

/** @addtogroup Debug Debugging support
 *  @{
 */

/**
 * @brief Set the rate at which the debug() user function is called
 * @param hz Rate in Hz
 */
void setDebugRate(int hz);

/** @} */ // end of addtogroup

/** @addtogroup Simulation Simulation-specific
 *  @{
 */

typedef void (*ResetPoseType)(void *);

/**
 * @brief Set a URDF joint to a position (before applying constraints or starting SDK control). You must call this for all the joints.
 * 
 * @param jointName URDF joint name
 * @param targetPos Position of joint in radians
 * @return true Joint was found
 * @return false Joint not found
 */
bool simResetJointState(std::string jointName, float targetPos);

/**
 * @brief Call between init() and begin() to use a custom URDF file (alpha)
 * 
 * @param filename custom URDF in the urdf/ directory. e.g. if there is a file urdf/myrobot.urdf, call `setCustomURDF("myrobot.urdf")`
 * @param resetPoseFunction Function to call after loading URDF to set the initial joint positions. You must call simResetJointState for each joint the SDK can use.
 * @param resetPoseParam Will be called as `resetPoseFunction(resetPoseParam)`
 * @param hoisted set to true to fix the base
 */
void simSetCustomURDF(std::string filename, ResetPoseType resetPoseFunction, void *resetPoseParam, bool hoisted = false);

enum ConstraintType
{
	// ConstraintType_REVOLUTE = 0,
	ConstraintType_PRISMATIC = 1,
	// ConstraintType_SPHERICAL = 2,
	// ConstraintType_PLANAR = 3,
	ConstraintType_FIXED = 4,
	ConstraintType_POINT2POINT = 5,
};

/**
 * @brief Create a constraint between two URDF joints (alpha)
 * 
 * @param parentJointName URDF joint name of parent joint
 * @param childJointName  URDF joint name of child joint
 * @param parentPoint  Parent link-frame coordinates of the attachment point
 * @param childPoint Child link-frame coordinates of the attachment point
 * @return int 
 */
int simConstrainLinks(std::string parentJointName, std::string childJointName, const float parentPoint[/* 3 */], const float childPoint[/* 3 */]);
/**
 * @brief Create a constraint between the world and the object (does not work yet)
 * 
 * @param type Constraint type (only a few are supported)
 * @param parentPoint 
 * @param childPoint 
 * @param jointAxis 
 * @return int 
 */
int simConstrainBase(ConstraintType type, const float parentPoint[/* 3 */], const float childPoint[/* 3 */], const float jointAxis[/* 3 */]);

/** @} */ // end of addtogroup

#endif
