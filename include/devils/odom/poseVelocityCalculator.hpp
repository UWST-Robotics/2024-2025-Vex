#pragma once

#include "devils/geometry/pose.hpp"
#include "pros/rtos.hpp"

namespace devils
{
    class PoseVelocityCalculator
    {
    public:
        /**
         * Gets the linear velocity of the robot in inches per second.
         * @return The linear velocity of the robot in inches per second.
         */
        virtual Vector2 &getVelocity()
        {
            return linearVelocity;
        }

        /**
         * Gets the angular velocity of the robot in radians per second.
         * @return The angular velocity of the robot in radians per second.
         */
        virtual double getAngularVelocity()
        {
            return angularVelocity;
        }

    protected:
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
            linearVelocity.x = positionDelta.x / dt;
            linearVelocity.y = positionDelta.y / dt;

            // Calculate Angular Velocity
            double angleDelta = pose.rotation - lastPose.rotation;
            angularVelocity = angleDelta / dt;

            // Update Last Pose
            lastPose = pose;
        }

    private:
        // Current state
        Vector2 linearVelocity = Vector2();
        double angularVelocity = 0;

        // Last state
        uint32_t lastTimestamp = 0;
        Pose lastPose = Pose();
    };
}
