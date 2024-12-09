#pragma once
#include "common/autoStepList.hpp"
#include "autoJumpToStep.hpp"
#include "autoDriveStep.hpp"
#include "autoDriveToStep.hpp"
#include "autoRotateStep.hpp"
#include "autoRotateToStep.hpp"
#include "autoRotateTowardStep.hpp"

namespace devils
{

    /**
     * Converts relative AutoSteps to absolute steps.
     */
    class AbsoluteStepConverter
    {
    public:
        /**
         * Tracks the pose over time to convert relative steps to absolute steps to improve reliability.
         *
         * `AutoDriveStep` >>> `AutoDriveToStep`
         *
         * `AutoRotateStep` >>> `AutoRotateToStep`
         *
         * @param autoStepList The list of relative steps to convert.
         * @return The list of absolute steps.
         */
        static AutoStepList relativeToAbsolute(AutoStepList &autoStepList)
        {
            // Get Step List
            std::vector<IAutoStep *> &steps = autoStepList.getAllSteps();
            std::vector<IAutoStep *> newSteps = std::vector<IAutoStep *>();
            newSteps.reserve(steps.size());

            // Track position over time
            Pose currentPose = Pose(0, 0, 0);

            // Iterate through the steps and convert them to absolute steps
            for (auto step : steps)
            {
                /*
                    ======== Step Conversion ========
                 */

                // Relative Drive Step
                AutoDriveStep *driveStep = dynamic_cast<AutoDriveStep *>(step);
                if (driveStep != nullptr)
                {
                    // Convert to Absolute
                    currentPose = Pose(
                        currentPose.x + driveStep->distance * std::cos(currentPose.rotation),
                        currentPose.y + driveStep->distance * std::sin(currentPose.rotation),
                        currentPose.rotation);

                    // Create Rotate To Step
                    // AutoRotateTowardStep *newRotateStep = new AutoRotateTowardStep(driveStep->chassis, driveStep->odomSource, currentPose);
                    // newSteps.push_back(newRotateStep);

                    // Create Drive To Step
                    AutoDriveToStep *newDriveStep = new AutoDriveToStep(driveStep->chassis, driveStep->odomSource, currentPose, driveStep->options);
                    newSteps.push_back(newDriveStep);

                    continue;
                }

                // Relative Rotate Step
                AutoRotateStep *rotateStep = dynamic_cast<AutoRotateStep *>(step);
                if (rotateStep != nullptr)
                {
                    // Convert to Absolute
                    currentPose = Pose(
                        currentPose.x,
                        currentPose.y,
                        currentPose.rotation + rotateStep->distance);

                    // Create Rotate To Step
                    AutoRotateToStep *newStep = new AutoRotateToStep(rotateStep->chassis, rotateStep->odomSource, currentPose.rotation, rotateStep->options);
                    newSteps.push_back(newStep);
                    continue;
                }

                /*
                    ======== Pose Changes =======
                */

                // Jump To Step
                AutoJumpToStep *jumpStep = dynamic_cast<AutoJumpToStep *>(step);
                if (jumpStep != nullptr)
                {
                    // Update Current Pose
                    currentPose = jumpStep->targetPose;

                    // Add Jump Step
                    newSteps.push_back(jumpStep);
                    continue;
                }

                // Absolute Rotate Step
                AutoRotateToStep *rotateToStep = dynamic_cast<AutoRotateToStep *>(step);
                if (rotateToStep != nullptr)
                {
                    // Update Current Pose
                    currentPose = Pose(currentPose.x, currentPose.y, rotateToStep->targetAngle);

                    // Add Rotate To Step
                    newSteps.push_back(rotateToStep);
                    continue;
                }

                // Absolute Drive Step
                AutoDriveToStep *driveToStep = dynamic_cast<AutoDriveToStep *>(step);
                if (driveToStep != nullptr)
                {
                    // Update Current Pose
                    currentPose = driveToStep->targetPose;

                    // Add Drive To Step
                    newSteps.push_back(driveToStep);
                    continue;
                }

                // Other Step
                newSteps.push_back(step);
            }

            // Return New Step List
            return AutoStepList(newSteps, autoStepList.loopCount);
        }
    };
}