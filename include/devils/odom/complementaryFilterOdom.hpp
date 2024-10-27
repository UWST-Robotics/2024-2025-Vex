#pragma once
#include "pros/gps.hpp"
#include "odomSource.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/units.hpp"
#include "../utils/logger.hpp"
#include "../utils/runnable.hpp"

namespace devils
{
    /**
     * Represents a set of odometry sources fused together using the complementary filter.
     */
    class ComplementaryFilterOdom : public OdomSource, public Runnable
    {
    public:
        /**
         * Creates a new fused odometry source using the complementary filter.
         * @param absoltueOdom The absolute odometry source
         * @param relativeOdom The relative odometry source
         * @param absoluteWeight The weight of the absolute odometry source
         */
        ComplementaryFilterOdom(OdomSource *absoluteOdom, OdomSource *relativeOdom, double absoluteWeight)
            : absoluteOdom(absoluteOdom),
              relativeOdom(relativeOdom),
              absoluteWeight(absoluteWeight),
              relativeWeight(1 - absoluteWeight)
        {
            if (absoluteWeight < 0 || absoluteWeight > 1)
                throw std::invalid_argument("Absolute weight must be between 0 and 1");
        }

        /**
         * Updates the odometry with the latest data from the sources
         */
        void update() override
        {
            // Get the current pose from each source
            Pose absolutePose = absoluteOdom->getPose();
            Pose relativePose = relativeOdom->getPose();

            // Check if the absolute pose has changed
            if (absolutePose.x != lastAbsolutePose.x || absolutePose.y != lastAbsolutePose.y) // Ignore GPS IMU
            {
                // Update the last absolute pose
                lastAbsolutePose = absolutePose;

                // Weight the absolute and relative poses
                currentPose = absolutePose * absoluteWeight + relativePose * relativeWeight;

                // Weight the absolute and relative rotations.
                // Accounts for normalization in the rotation.
                // double diffRad = Units::diffRad(absolutePose.rotation, relativePose.rotation);
                // currentPose.rotation = Units::normalizeRadians(relativePose.rotation + diffRad * absoluteWeight);

                // Use IMU for rotation
                currentPose.rotation = relativePose.rotation;

                // Update Relative Odometry
                relativeOdom->setPose(currentPose);
            }
            else
            {
                // Apply the relative pose
                currentPose = relativePose;
            }
        }

        /**
         * Gets the current pose of the robot
         * @return The current pose of the robot
         */
        Pose &getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot
         * @param pose The pose to set the robot to
         */
        void setPose(Pose &pose) override
        {
            currentPose = pose;
            absoluteOdom->setPose(pose);
            relativeOdom->setPose(pose);
        }

    private:
        OdomSource *absoluteOdom;
        OdomSource *relativeOdom;
        double absoluteWeight;
        double relativeWeight;

        Pose lastAbsolutePose = Pose(0, 0, 0);
        Pose currentPose = Pose(0, 0, 0);
    };
}