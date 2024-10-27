#pragma once
#include "../chassis/chassis.hpp"
#include "../path/generatedPath.hpp"
#include "../path/pathFile.hpp"
#include "../odom/odomSource.hpp"
#include "../utils/logger.hpp"
#include "../utils/pid.hpp"
#include "autoController.hpp"
#include "../geometry/lerp.hpp"
#include <cmath>
#include <vector>

namespace devils
{
    /**
     * Controller for inputting a control for a set amount of time (Open-loop)
     */
    class TimeController : public AutoController
    {
    public:
        /**
         * Constructs a new TimeController.
         * @param chassis The chassis to control.
         * @param duration The duration to run the controller for in milliseconds.
         * @param forward The forward value to set, from -1 to 1.
         * @param turn The turn value to set, from -1 to 1.
         * @param strafe The strafe value to set, from -1 to 1.
         */
        TimeController(
            BaseChassis &chassis,
            int duration,
            double forward = 0,
            double turn = 0,
            double strafe = 0)
            : chassis(chassis),
              duration(duration),
              forward(forward),
              turn(turn),
              strafe(strafe),
              targetPose(forward, strafe, turn)
        {
        }

        void reset() override
        {
            AutoController::reset();
            startTime = pros::millis();
        }

        void update() override
        {
            // Check if not already reset
            if (startTime < 0)
                reset();

            // Update State
            currentState.isFinished = startTime > 0 && pros::millis() - startTime > duration;

            // Drive
            if (getFinished())
                chassis.stop();
            else
                chassis.move(forward, turn, strafe);
        }

        /**
         * Sets the speeds for the controller.
         * @param forward The forward value to set, from -1 to 1.
         * @param turn The turn value to set, from -1 to 1.
         * @param strafe The strafe value to set, from -1 to 1.
         */
        void setSpeeds(double forward, double turn, double strafe = 0)
        {
            this->forward = forward;
            this->turn = turn;
            this->strafe = strafe;
        }

        /**
         * Sets the duration for the controller.
         * @param duration The duration to run the controller for in milliseconds.
         */
        void setDuration(int duration)
        {
            this->duration = duration;
        }

    private:
        // Input Values
        BaseChassis &chassis;
        int duration;
        double forward;
        double turn;
        double strafe;

        // State Values
        Pose targetPose = Pose(0, 0, 0);
        int startTime = -1;
    };
}