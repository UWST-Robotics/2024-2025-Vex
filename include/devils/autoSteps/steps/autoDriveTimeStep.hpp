#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../autoStep.hpp"
#include "../../utils/math.hpp"
#include "../../utils/timer.hpp"
#include "../../chassis/chassisBase.hpp"

namespace devils
{
    /**
     * Drives the robot for a given duration in open loop.
     */
    class AutoDriveTimeStep : public AutoStep
    {
    public:
        /**
         * Drives the robot for a given duration in open loop.
         * @param chassis The chassis to control.
         * @param duration Time to drive in milliseconds.
         * @param forwardSpeed The forward speed of the robot from -1 to 1.
         * @param turnSpeed The turn speed of the robot from -1 to 1.
         * @param strafeSpeed The strafe speed of the robot from -1 to 1.
         */
        AutoDriveTimeStep(
            ChassisBase &chassis,
            uint32_t duration,
            double forwardSpeed,
            double turnSpeed,
            double strafeSpeed = 0)
            : chassis(chassis),
              timer(duration),
              forwardSpeed(forwardSpeed),
              turnSpeed(turnSpeed),
              strafeSpeed(strafeSpeed)
        {
        }

    protected:
        void onStart() override
        {
            // Start Timer
            timer.start();
        }

        void onUpdate() override
        {
            // Set Speed
            chassis.move(forwardSpeed, turnSpeed, strafeSpeed);
        }

        void onStop() override
        {
            // Stop Chassis
            chassis.stop();
        }

        bool checkFinished() override
        {
            return timer.finished();
        }

        // State
        Timer timer;

        // Params
        ChassisBase &chassis;
        double forwardSpeed = 0;
        double turnSpeed = 0;
        double strafeSpeed = 0;
    };
}