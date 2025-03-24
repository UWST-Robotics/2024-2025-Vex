#pragma once

#include "devils/geometry/pose.hpp"
#include "pros/rtos.hpp"

namespace devils
{
    class PoseVelocityCalculator
    {
    public:
        /**
         * Gets current velocity of the robot.
         * @return The current velocity of the robot as a `PoseVelocity`.
         */
        virtual PoseVelocity &getVelocity()
        {
            return currentVelocity;
        }

    protected:
        /**
         * Updates the current velocity of the robot.
         * Should be run whenever the current `pose` is updated.
         * @param pose The current pose of the robot.
         */
        void updateVelocity(Pose pose)
        {
            // Calculate Time Delta
            uint32_t timestamp = pros::millis();
            double dt = (timestamp - lastTimestamp) / 1000.0;

            // Initial update
            if (lastTimestamp == 0)
            {
                lastTimestamp = timestamp;
                lastPose = pose;
                return;
            }

            // Skip if no time has passed
            if (dt <= 0)
                return;

            // Update Timestamp
            lastTimestamp = timestamp;

            // Calculate Linear Velocity
            Vector2 positionDelta = pose - lastPose;
            double linearVelocityX = positionDelta.x / dt;
            double linearVelocityY = positionDelta.y / dt;

            // Calculate Angular Velocity
            double angleDelta = pose.rotation - lastPose.rotation;
            double angularVelocity = angleDelta / dt;

            // Update Velocity
            currentVelocity = PoseVelocity(linearVelocityX, linearVelocityY, angularVelocity);

            // Update Last Pose
            lastPose = pose;
        }

    private:
        // Current state
        PoseVelocity currentVelocity = PoseVelocity();

        // Last state
        uint32_t lastTimestamp = 0;
        Pose lastPose = Pose();
    };
}
