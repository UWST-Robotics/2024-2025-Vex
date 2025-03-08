#pragma once
#include "pros/rtos.hpp"
#include "../common/autoStep.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../../odom/odomSource.hpp"
#include "../../path/path.hpp"
#include "autoDriveToStep.hpp"

namespace devils
{
    /**
     * Represents a step to follow a path using basic pure pursuit.
     */
    class AutoPurePursuitStep : public AutoDriveToStep
    {
    public:
        /**
         * Creates a new Pure Pursuit step.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param path The path to follow.
         */
        AutoPurePursuitStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            Path *path,
            Options options = Options::defaultOptions)
            : AutoDriveToStep(chassis, odomSource, Pose(0, 0, 0), options),
              odomSource(odomSource),
              path(path)
        {
        }

        void onStart() override
        {
            // Start the drive step
            AutoDriveToStep::onStart();
        }

        void onUpdate() override
        {
            // Get the current pose
            Pose currentPose = odomSource.getPose();

            // Find the closest index on the path
            double closestIndex = 0;
            double closestDistance = std::numeric_limits<double>::infinity();
            for (double i = 0; i < path->getLength(); i += DELTA_T)
            {
                // Get the distance to the path at i
                Pose pathPose = path->getPoseAt(i);
                double distance = currentPose.distanceTo(pathPose);

                // Check if this is the closest point
                if (distance < closestDistance)
                {
                    closestDistance = distance;
                    closestIndex = i;
                }
            }

            // Get the lookahead index
            double lookaheadIndex = closestIndex;
            while (lookaheadIndex < path->getLength() &&
                   currentPose.distanceTo(path->getPoseAt(lookaheadIndex)) < LOOKAHEAD_DIST)
            {
                lookaheadIndex += DELTA_T;
            }

            // Set the target pose
            Pose lookaheadPose = path->getPoseAt(lookaheadIndex);
            AutoDriveToStep::targetPose = lookaheadPose;

            // Update the drive step
            AutoDriveToStep::onUpdate();
        }

        void onStop() override
        {
            // Stop the drive step
            AutoDriveToStep::onStop();
        }

        bool checkFinished() override
        {
            return AutoDriveToStep::checkFinished();
        }

    protected:
        static constexpr double DELTA_T = 0.01;       // indices
        static constexpr double LOOKAHEAD_DIST = 8.0; // inches

        OdomSource &odomSource;
        Path *path;
    };
}