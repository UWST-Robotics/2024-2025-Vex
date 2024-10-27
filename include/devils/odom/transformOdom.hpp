#pragma once
#include "../chassis/tankChassis.hpp"
#include "../hardware/imu.hpp"
#include "../hardware/rotationSensor.hpp"
#include "../utils/logger.hpp"
#include "../geometry/pose.hpp"
#include "odomSource.hpp"
#include "pros/rtos.hpp"
#include "pros/error.h"
#include <cmath>
#include <errno.h>

namespace devils
{
    /**
     * Represents an odometry system that transforms another odometry system as needed.
     */
    class TransformOdom : public OdomSource
    {
    public:
        /**
         * Creates a new transform odometry system.
         * @param odomSource The odometry source to transform.
         * @param rotationOffset Amount of rotational offset in degrees
         */
        TransformOdom(OdomSource &odomSource, double rotationOffset)
            : odomSource(odomSource),
              rotationOffset(rotationOffset)
        {
        }

        /**
         * Gets the current pose of the robot.
         */
        Pose &getPose() override
        {
            Pose &gpsPose = odomSource.getPose();
            double newX = std::cos(rotationOffset) * gpsPose.x + std::sin(rotationOffset) * gpsPose.y;
            double newY = std::cos(rotationOffset) * gpsPose.y + std::sin(rotationOffset) * gpsPose.x;

            currentPose.x = newX;
            currentPose.y = newY;
            currentPose.rotation = Units::normalizeRadians(gpsPose.rotation + rotationOffset);

            return currentPose;
        }

        /**
         * Sets the current pose of the robot.
         * @param pose The pose to set the robot to.
         */
        void setPose(Pose &pose) override
        {
            Pose newPose = pose;
            double newX = std::cos(-rotationOffset) * pose.x + std::sin(-rotationOffset) * pose.y;
            double newY = std::cos(-rotationOffset) * pose.y + std::sin(-rotationOffset) * pose.x;

            newPose.x = newX;
            newPose.y = newY;
            newPose.rotation = Units::normalizeRadians(pose.rotation - rotationOffset);

            odomSource.setPose(newPose);
        }

        /**
         * Sets the rotational offset
         * @param rotationOffset - Rotational offset in radians
         */
        void setRotation(double rotationOffset)
        {
            this->rotationOffset = rotationOffset;
        }

    private:
        Pose currentPose = Pose();
        OdomSource &odomSource;

        double rotationOffset = 0;
    };
}