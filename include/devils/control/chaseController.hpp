#pragma once
#include "autoController.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/polygon.hpp"
#include "../path/occupancyGrid.hpp"
#include "../path/pathFinder.hpp"
#include "../odom/odomSource.hpp"
#include "../gameobject/gameObjectManager.hpp"
#include "../control/findController.hpp"
#include "../control/directController.hpp"
#include "../hardware/opticalSensor.hpp"
#include "../hardware/visionSensor.hpp"

namespace devils
{
    /**
     * Controller chases to the closest game object and collects it.
     */
    class ChaseController : public AutoController
    {
    public:
        /**
         * Constructs a new collection controller.
         * @param chassis The chassis to control.
         * @param odometry The odometry source to use.
         * @param gameObjectManager The game objects to collect.
         * @param occupancyGrid The occupancy grid to use.
         */
        ChaseController(BaseChassis &chassis, OdomSource &odometry, VisionSensor &visionSensor)
            : chassis(chassis),
              odometry(odometry),
              visionSensor(visionSensor),
              directController(chassis, odometry)
        {
        }

        void reset() override
        {
            AutoController::reset();
            directController.reset();
        }

        void update() override
        {
            // Get Current State
            Pose currentPose = odometry.getPose();
            auto visionObjects = visionSensor.getObjects();

            // Get Objects within Area
            bool foundObject = false;
            for (auto object : visionObjects)
            {
                Pose absolutePose = _approximateObjectPose(currentPose, object);
                if (collectionArea != nullptr && !collectionArea->contains(absolutePose))
                    continue;
                foundObject = true;
                targetPose = absolutePose;
            }

            // End if no objects on field
            if (!foundObject)
            {
                currentState.isFinished = true;
                chassis.stop();
                return;
            }

            // Get the closest object
            currentState.target = &targetPose;

            // Check if object is picked up
            bool hasObject = false;
            double objectDistance = targetPose.distanceTo(currentPose);
            if (storageSensor != nullptr)
                hasObject = storageSensor->getProximity() > OPTICAL_PROXIMITY;
            else
                hasObject = objectDistance < COLLECTION_DISTANCE;

            // Mark object as collected
            if (hasObject)
            {
                currentState.isFinished = true;
                chassis.stop();
                return;
            }

            // Chase the object
            directController.setTargetPose(targetPose);
            directController.update();

            // Update State
            currentState.events = &CHASE_EVENTS;
        }

        /**
         * Checks if the robot has objects in its vision.
         * @return True if objects are in vision, false otherwise.
         */
        bool hasObjectsInVision()
        {
            // Get Current State
            Pose currentPose = odometry.getPose();
            auto visionObjects = visionSensor.getObjects();

            // Get Objects within Area
            for (auto object : visionObjects)
            {
                Pose absolutePose = _approximateObjectPose(currentPose, object);
                if (collectionArea != nullptr && !collectionArea->contains(absolutePose))
                    continue;
                return true;
            }
            return false;
        }

        Pose _approximateObjectPose(Pose currentPose, VisionObject object)
        {
            // Approximate Distance w/ Object Size
            double objectSize = std::sqrt(object.area);
            double objectDistance = 770 * std::pow(objectSize, -0.75);
            objectDistance = std::clamp(objectDistance, 0.0, 24.0);

            // Approximate Angle w/ Object X position & Camera FOV
            double objectAngle = visionSensor.getAngle(object);

            // Calculate Pose
            Pose relativePose = Pose(
                objectDistance * cos(objectAngle),
                objectDistance * sin(objectAngle),
                objectAngle);
            return currentPose + relativePose;
        }

        /**
         * Uses an optical sensor to detect game objects in the robot's collection.
         * @param sensor The optical sensor to use.
         */
        void useCollectionSensor(OpticalSensor *sensor)
        {
            storageSensor = sensor;
        }

        /**
         * Uses a polygon to define the area where game objects can be collected.
         * @param polygon The area to use.
         */
        void useCollectionArea(Polygon *polygon)
        {
            collectionArea = polygon;
        }

    private:
        // Constants
        static constexpr double COLLECTION_DISTANCE = 12.0; // in
        static constexpr double OPTICAL_PROXIMITY = 0.9;    // %
        PathEvents CHASE_EVENTS = {PathEvent("chase", "")};

        // Required Components
        BaseChassis &chassis;
        OdomSource &odometry;
        VisionSensor &visionSensor;

        // State
        Pose targetPose = Pose();
        DirectController directController;
        bool isChasing = false;

        // Optional Components
        OpticalSensor *storageSensor = nullptr;
        Polygon *collectionArea = nullptr;
    };
}