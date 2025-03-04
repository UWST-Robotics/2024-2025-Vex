#pragma once
#include <string>
#include "pros/rtos.hpp"
#include "../common/autoStep.hpp"
#include "../../utils/math.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "autoDriveToStep.hpp"
#include "../../hardware/camera.hpp"
#include "../../geometry/units.hpp"

namespace devils
{
    /**
     * Searches for vision targets and drives to them.
     */
    class AutoDriveToVisionStep : public AutoDriveToStep
    {
    public:
        /**
         * Searches for vision targets and drives to them.
         * @param chassis The chassis to control.
         * @param odomSource The odometry source to use.
         * @param camera The camera to use for vision.
         * @param defaultPose The default pose to use if no vision targets are found.
         * @param objectSizeMultiplier Constant used to scale the object size to the target distance (px / in)
         * @param targetOptions The options for the drive step if a target is found.
         * @param defaultOptions The options for the drive step if no target is found.
         */
        AutoDriveToVisionStep(
            ChassisBase &chassis,
            OdomSource &odomSource,
            ICamera &camera,
            Pose defaultPose,
            double objectSizeMultiplier = 1.0,
            Options targetOptions = Options::defaultOptions,
            Options defaultOptions = Options::defaultOptions)
            : AutoDriveToStep(chassis, odomSource, Pose(0, 0, 0), defaultOptions),
              camera(camera),
              defaultPose(defaultPose),
              objectSizeMultiplier(objectSizeMultiplier),
              targetOptions(targetOptions),
              defaultOptions(defaultOptions)
        {
        }

        void onStart() override
        {
            updateVisionState();
            AutoDriveToStep::onStart();
        }

        /**
         * Fetches the objects in the frame and updates the target pose.
         * If no objects are found, the default pose is used.
         */
        void updateVisionState()
        {
            // Get the objects in the frame
            std::vector<CameraTarget> objects = camera.getObjectsInFrame();

            // If no objects are found, drive to the default pose
            if (objects.size() == 0)
            {
                AutoDriveToStep::targetPose = defaultPose;
                AutoDriveToStep::options = defaultOptions;
                return;
            }

            // Get the first object
            CameraTarget target = objects[0];

            // Calculate the target pose
            Pose objectPose = getObjectPose(target);

            // Drive to the target pose
            AutoDriveToStep::targetPose = objectPose;
            AutoDriveToStep::options = targetOptions;
        }

    protected:
        /**
         * Gets the pose of the object in the frame.
         * @param target The target to get the pose of.
         * @return The pose of the object in the frame.
         */
        Pose getObjectPose(CameraTarget target)
        {
            // Get current pose
            Pose currentPose = odomSource.getPose();

            // Get camera properties
            double cameraFOV = camera.getFOV();
            Vector2 cameraSize = camera.getSize();

            // Get the object center
            double targetCenterX = target.x + target.width / 2;
            double targetCenterY = target.y + target.height / 2;

            // Get the angle to the target
            double targetAngle = (targetCenterX - cameraSize.x * 0.5) / cameraSize.x * cameraFOV;

            // Get the distance to the target
            double targetArea = target.width * target.height;
            double targetDistance = objectSizeMultiplier / sqrt(targetArea);

            // Calculate the target pose
            Pose objectPose = Pose(
                targetDistance * cos(targetAngle + currentPose.rotation) + currentPose.x,
                targetDistance * sin(targetAngle + currentPose.rotation) + currentPose.y,
                currentPose.rotation);

            return objectPose;
        }

        // Drive Step Variables
        Pose defaultPose;
        ICamera &camera;
        Options defaultOptions;
        Options targetOptions;
        double objectSizeMultiplier;
    };
}