#pragma once

#include "visionTargetProvider.h"
#include "../hardware/structs/camera.h"
#include "../odom/odomSource.hpp"
#include "../geometry/units.hpp"

namespace devils
{
    class GroundVisionProvider : public IVisionTargetProvider
    {
    public:
        struct Options
        {
            /// @brief Minimum area of the target in pixels.
            double minArea = 0.0;

            /// @brief Maximum area of the target in pixels.
            double maxArea = 1000000.0;

            /// @brief Constant used to calculate the distance to the target from the area.
            double areaToDistance = 1.0;

            /// @brief The default options for the drive step.
            static Options defaultOptions;
        };

        /**
         * Creates a new ground vision provider.
         * @param camera The camera to use.
         * @param options The options for the ground vision provider.
         */
        GroundVisionProvider(ICamera *camera, OdomSource &odomSource, Options options = Options::defaultOptions)
            : camera(camera),
              odomSource(odomSource),
              options(options)
        {
        }

        bool hasTargets() override
        {
            update();
            return this->hasTargetsState;
        }

        Pose getClosestTarget() override
        {
            update();
            return this->closestTargetState;
        }

    protected:
        /**
         * Updates the `hasTargets` and `closestTarget` state.
         */
        void update()
        {
            // Get the objects in view
            auto objectsInView = camera->getObjectsInView();

            // Get camera parameters
            ICamera::Parameters cameraParams = camera->getParameters();

            // Get current pose
            Pose currentPose = odomSource.getPose();

            // Find the closest target
            Pose closestTarget = Pose();
            double closestTargetArea = 0;
            for (auto &object : objectsInView)
            {
                // Filter by area
                double area = object.width * object.height;
                if (area < options.minArea || area > options.maxArea)
                    continue;

                // Check if the target is closer
                if (area < closestTargetArea)
                    continue;

                // Calculate Pose
                double distance = options.areaToDistance / std::sqrt(area);
                double angle = (object.x - cameraParams.imageWidth / 2) * cameraParams.xFOV;

                Pose targetPose = Pose(
                    distance * std::cos(angle) + currentPose.x,
                    distance * std::sin(angle) + currentPose.y,
                    currentPose.rotation + angle);

                // Update closest target
                closestTarget = targetPose;
                closestTargetArea = area;
            }

            // Check if we found a target
            if (closestTargetArea == 0)
            {
                hasTargetsState = false;
                return;
            }

            // Update state
            hasTargetsState = true;
            this->closestTargetState = closestTarget;
        }

    private:
        // State
        Pose closestTargetState = Pose();
        bool hasTargetsState = false;

        // Params
        ICamera *camera;
        OdomSource &odomSource;
        Options options;
    };

    // Define the default options
    GroundVisionProvider::Options GroundVisionProvider::Options::defaultOptions = GroundVisionProvider::Options();
}