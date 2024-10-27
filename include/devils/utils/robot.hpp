#pragma once

namespace devils
{
    /**
     * Represents a robot control system.
     * Can be swapped out for different robots at compile time
     * by modifying the type defined in `main.cpp` file.
     */
    struct Robot
    {
        /**
         * Ran when the FMS is connected.
         */
        virtual void competition() {}

        /**
         * Ran when the robot is disabled with the FMS connected.
         */
        virtual void disabled() {}

        /**
         * Ran at the start of the Autonomous period.
         */
        virtual void autonomous() {}

        /**
         * Ran at the start of the Teleoperated period.
         */
        virtual void opcontrol() {}

        /// @brief The main game controller.
        pros::Controller mainController = pros::Controller(pros::E_CONTROLLER_MASTER);

        /// @brief The partner game controller.
        pros::Controller partnerController = pros::Controller(pros::E_CONTROLLER_PARTNER);
    };
}