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
     * Controller for driving directly to a point using Odometry
     */
    class DirectController : public AutoController
    {
    public:
        /**
         * Constructs a new PursuitController.
         * @param chassis The chassis to control.
         * @param odometry The odometry source to use.
         */
        DirectController(BaseChassis &chassis, OdomSource &odometry)
            : chassis(chassis),
              odometry(odometry)
        {
        }

        void update() override
        {
            // Get Current Pose
            Pose currentPose = odometry.getPose();

            // Calculate Forward & Turn
            double deltaX = targetPose->x - currentPose.x;
            double deltaY = targetPose->y - currentPose.y;
            double distanceToPose = sqrt(pow(deltaX, 2) + pow(deltaY, 2)); // Prevent sharp turns when close
            double deltaRotation = Units::diffRad(atan2(deltaY, deltaX), currentPose.rotation);
            double deltaForward = cos(currentPose.rotation) * deltaX + sin(currentPose.rotation) * deltaY;

            // Auto Reverse
            if (autoReverse)
                isReversed = deltaForward < 0;

            // Handle Reversed
            if (isReversed)
                deltaRotation = Units::diffRad(deltaRotation, M_PI);

            // Calculate PID
            double forward = translationPID.update(-deltaForward);
            double turn = rotationPID.update(-deltaRotation);

            // Clamp Values
            if (isReversed)
                forward = std::clamp(forward, -1.0, 0.0);
            else
                forward = std::clamp(forward, 0.0, 1.0);
            turn = std::clamp(turn * distanceToPose, -1.0, 1.0);

            // Drive
            chassis.move(forward, turn);
        }

        /**
         * Reverse the robot's direction.
         * @param isReversed Whether the robot is driving in reverse.
         */
        void setReverse(bool isReversed)
        {
            this->isReversed = isReversed;
        }

        /**
         * Sets whether the robot should automatically reverse when driving backwards.
         * @param autoReverse Whether the robot should automatically reverse.
         */
        void setAutoReverse(bool autoReverse)
        {
            this->autoReverse = autoReverse;
        }

        /**
         * Sets the target pose for the controller.
         */
        void setTargetPose(Pose &targetPose)
        {
            this->targetPose = &targetPose;
            currentState.target = &targetPose;
        }

    private:
        PID translationPID = PID(0.18, 0, 0); // <-- Translation
        PID rotationPID = PID(0.05, 0, 0);    // <-- Rotation

        // Object Handles
        BaseChassis &chassis;
        OdomSource &odometry;

        // State
        Pose *targetPose;
        bool isReversed = false;
        bool autoReverse = false;
    };
}