#pragma once
#include "autoController.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/polygon.hpp"
#include "../path/occupancyGrid.hpp"
#include "../path/pathFinder.hpp"
#include "../odom/odomSource.hpp"
#include "../gameobject/gameObjectManager.hpp"
#include "../control/pursuitController.hpp"
#include "../control/directController.hpp"

namespace devils
{
    /**
     * Controller for retriving game objects from the field.
     */
    class FindController : public AutoController
    {
    public:
        /**
         * Constructs a new collection controller.
         * @param chassis The chassis to control.
         * @param odometry The odometry source to use.
         * @param occupancyGrid The occupancy grid to use.
         */
        FindController(BaseChassis &chassis, OdomSource &odometry, OccupancyGrid &occupancyGrid)
            : controller(chassis, odometry, nullptr, true),
              chassis(chassis),
              odometry(odometry),
              occupancyGrid(occupancyGrid),
              targetPose(odometry.getPose())
        {
        }

        void reset() override
        {
            AutoController::reset();
            controller.reset();
            isPathGenerated = false;
        }

        void update() override
        {
            controller.update();
        }

        AutoState &getState() override
        {
            return controller.getState();
        }

        /**
         * Sets the target pose for the controller.
         * Regenerates the path if the target is too far away from the current path.
         * @param targetPose The target pose to set.
         */
        void setTargetPose(Pose targetPose)
        {
            this->targetPose = targetPose;
            currentState.target = &targetPose;

            // Recalculate path if the target is too far away
            if (REGENERATE_DISTANCE < targetPose.distanceTo(originalTargetPose) || !isPathGenerated)
            {
                // Set Target
                originalTargetPose = targetPose;

                // Get Current Pose
                Pose currentPose = odometry.getPose();

                // Regenerate the path
                currentPath = PathFinder::generatePath(currentPose, targetPose, occupancyGrid);

                // Update Pursuit Controller
                controller.setPath(&currentPath);

                // Update State
                isPathGenerated = true;
            }
        }

    private:
        static constexpr double REGENERATE_DISTANCE = 12.0; // in

        // Required Components
        BaseChassis &chassis;
        OdomSource &odometry;
        OccupancyGrid &occupancyGrid;

        // State
        PursuitController controller;
        GeneratedPath currentPath;
        Pose targetPose;
        Pose originalTargetPose;
        bool isPathGenerated = false;
    };
}