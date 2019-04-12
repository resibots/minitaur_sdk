/*
 * Copyright (C) Ghost Robotics - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 * Written by Avik De <avik@ghostrobotics.io>
 */
#ifndef Peripheral_h
#define Peripheral_h

/** @addtogroup Behavior Behavior
 *  @{
 */

//
/**
 * @brief Class for integrating peripheral sensors, tools, etc.
 */
class Peripheral {
public:
  /**
   * @brief Function called to initialize behavior / peripheral.
   *
   * @details This gets called when the behavior / peripheral is started. May be called multiple times, i.e. when switching behaviors.
   */
  virtual void begin() = 0;
  /**
   * @brief Function called repeatedly (at CONTROL_RATE) to update.
   *
   * @details The main logic for the behavior / peripheral goes here.
   */
  virtual void update() = 0;
};
/** @} */ // end of addtogroup

#endif
