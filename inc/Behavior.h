/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Behavior_h
#define Behavior_h

#include "Peripheral.h"
#include "simple.pb.h"
#include <stdint.h>


/** @addtogroup Behavior Behavior
 *  @{
 */

/**
 * @brief Abstract base class for implementing behaviors
 */
class Behavior : public Peripheral {
public:
  // should be one of LIMB or JOINT
//  RobotCommand_Mode mode;
  Behavior() /*: mode(RobotCommand_Mode_LIMB)*/ {}

	/**
	 * @brief Should return false if the task is complete (for switching to another task)
	 * @return True if running
	 */
  virtual bool running() { return false; }
  /**
   * @brief Called when a stop is requested by a BehaviorCmd
   */
  virtual void end() {}

//
#if defined(__clang__)

#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#elif defined(_MSC_VER)

#endif
  /**
   * @brief Send a signal to the behavior; for example a "leap" signal, or a "rollover" signal. These are typically used to send a short message to a running behavior about executing some kind of transition.
   * 
   * @param mode ::BehaviorMode_STOP and ::BehaviorMode_RUN are reserved, 2 and 3 are set by the joystick.
   */
  virtual void signal(uint32_t mode=0) {}
#if defined(__clang__)

#elif defined(__GNUC__) || defined(__GNUG__)
#pragma GCC diagnostic pop
#elif defined(_MSC_VER)

#endif

  // TODO Add composition requirements like a goalset description for sequential
};


/** @} */ // end of addtogroup


#endif
