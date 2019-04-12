/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef ReorientableBehavior_h
#define ReorientableBehavior_h

#include "Behavior.h"

/** @addtogroup Behavior Behavior
 *  @{
 */


/**
 * @brief Abstract class that inherits Behavior with automatic reorientation members
 */
class ReorientableBehavior : public Behavior
{
  // angleFromUpright is the spherical angle of the upright vector from the 
  // inertial upright vector [0,0,1]. It is between 0, pi
  float angleFromUpright = 0;
public:
  // Limit before which reorientating kicks in
  float ANGLE_LIMIT = 0.7;

  /**
   * This variable is set to true if the robot is upside down
   */
  bool bInverted = false;

  /**
   * @brief Depending on if inverted, returns if this leg is in the front or rear of the robot
   * 
   * @param legidx Index into ::limb, for quadruped 0--3 
   * @return True if it is a front leg
   */
  bool isFront(int legidx);
  /**
   * @brief Depending on if inverted, returns if this leg is in the right or left of the robot
   * 
   * @param legidx Index into ::limb, for quadruped 0--3 
   * @return True if it is a right leg
   */
  bool isRight(int legidx);

  /**
   * @brief Reorientation specific to a quadruped. This function calls the actual
   * reorientation update, and also sets bInverted. If it returns true, the 
   * behavior update() should return (legs are being controlled to reorient)
   * 
   * @return true if need to reorient, false if the behavior update() should continue
   */
  bool isReorienting();

  void reorientUpdate();
};

/** @} */ // end of addtogroup


#endif
