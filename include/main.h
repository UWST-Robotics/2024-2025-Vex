/**
 * \file main.h
 *
 * Contains common definitions and header files used throughout your PROS
 * project.
 *
 * \copyright Copyright (c) 2017-2023, Purdue University ACM SIGBots.
 * All rights reserved.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#pragma once
#define PROS_USE_SIMPLE_NAMES // Allows us to use pros::Motor instead of pros::c::motor
#define PROS_USE_LITERALS     // Allows us to use 200_ms instead of pros::c::millisecond_t(200)

// Add each robot definition here
#include "devils/2025/prototypeRobot.hpp"

using namespace devils;

/**
 * Prototypes for the competition control tasks are redefined here to ensure
 * that they can be called from user code (i.e. calling autonomous from a
 * button press in opcontrol() for testing purposes).
 */
extern "C"
{
    std::shared_ptr<Robot> robot;
    void autonomous(void);
    void initialize(void);
    void disabled(void);
    void competition_initialize(void);
    void opcontrol(void);
}