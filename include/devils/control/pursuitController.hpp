#pragma once
#include "../chassis/chassis.hpp"
#include "../path/generatedPath.hpp"
#include "../path/pathFile.hpp"
#include "../odom/odomSource.hpp"
#include "../utils/logger.hpp"
#include "../utils/pid.hpp"
#include "../geometry/lerp.hpp"
#include "autoController.hpp"
#include "directController.hpp"
#include <cmath>
#include <vector>

namespace devils
{
    /**
     * Controller for follwing a path with sensor feedback using Basic Pure Pursuit.
     */
    class PursuitController : public AutoController
    {
    public:
        /**
         * Constructs a new PursuitController.
         * @param chassis The chassis to control.
         * @param odometry The odometry source to use.
         * @param path The generated path to follow.
         * @param skipCheckpoints Whether the controller can skip checkpoints.
         */
        PursuitController(BaseChassis &chassis, OdomSource &odometry, GeneratedPath *path = nullptr, bool skipCheckpoints = false)
            : chassis(chassis),
              odometry(odometry),
              directController(chassis, odometry),
              skipCheckpoints(skipCheckpoints),
              currentPath(path)
        {
            setPath(path);
        }

        void reset() override
        {
            AutoController::reset();
            directController.reset();
            lookaheadPointIndex = 0;
            controlPointIndex = 0;
        }

        void update() override
        {
            // Abort if path is missing
            if (currentPath == nullptr || pathPoints == nullptr || controlPoints == nullptr)
                return;
            // Abort if finished
            if (currentState.isFinished)
                return;

            // Get Current Pose
            Pose currentPose = odometry.getPose();

            // Update Control Point Index
            ControlPoint *prevControlPoint = &controlPoints->at(controlPointIndex);
            int prevCheckpointPathIndex = currentPath->controlPointIndices.at(controlPointIndex);
            int checkpointPathIndex = currentPath->controlPointIndices.at(controlPointIndex + 1);

            // Update Lookahead Point Index
            Pose *targetPose = nullptr;
            for (int i = lookaheadPointIndex; i < pathPoints->size(); i++)
            {
                targetPose = &pathPoints->at(i);
                lookaheadPointIndex = i;
                if (targetPose->distanceTo(currentPose) >= lookaheadDistance)
                    break;
            }

            // Checkpoint
            bool isLookaheadPastCheckpoint = lookaheadPointIndex > checkpointPathIndex || lookaheadPointIndex >= pathPoints->size() - 1;
            // bool isRotated = std::abs(Units::diffRad(currentPose.rotation, targetPose->rotation)) < ROTATIONAL_THRESHOLD;
            if (isLookaheadPastCheckpoint)
            {
                // Increment Control Point
                if (controlPointIndex < controlPoints->size() - 2)
                {
                    controlPointIndex++;

                    // Re-run update to get new control point
                    update();
                    return;
                }
                // Finish
                else
                {
                    // Set State
                    currentState.isFinished = true;
                    currentState.events = &controlPoints->back().events;
                    currentState.debugText = "Finished Path " + std::to_string(currentState.events->size());

                    // Stop Chassis
                    chassis.stop();
                    return;
                }
            }

            // Update State
            currentState.target = targetPose;
            currentState.events = &prevControlPoint->events;
            currentState.debugText = "I=" + std::to_string(lookaheadPointIndex) + ", R=" + std::to_string(prevControlPoint->isReversed);
            Logger::debug(currentState.debugText);

            // Auto Reverse
            // bool closeToPrevCheckpoint = std::abs(lookaheadPointIndex - prevCheckpointPathIndex) < AUTO_REVERSE_INDICES;
            // bool closeToNextCheckpoint = std::abs(lookaheadPointIndex - checkpointPathIndex) < AUTO_REVERSE_INDICES;
            // directController.setAutoReverse(closeToPrevCheckpoint || closeToNextCheckpoint);
            directController.setAutoReverse(true);

            // Drive To Point
            directController.setTargetPose(*targetPose);
            directController.setReverse(prevControlPoint->isReversed);
            directController.update();
        }

        /**
         * Changes the path and resets the controller.
         * @param path The new path to follow.
         */
        void setPath(GeneratedPath *path)
        {
            currentPath = path;
            if (path != nullptr)
            {
                controlPoints = &currentPath->controlPoints;
                pathPoints = &currentPath->pathPoints;
            }
            reset();
        }

        /**
         * Sets the lookahead distance for the controller.
         * @param distance The lookahead distance to set, in inches.
         */
        void setLookaheadDistance(double distance)
        {
            lookaheadDistance = distance;
        }

    private:
        static constexpr double DEFAULT_LOOKAHEAD_DISTANCE = 8.0; // in

        // Object Handles
        BaseChassis &chassis;
        OdomSource &odometry;

        // Shorthands
        GeneratedPath *currentPath;
        ControlPoints *controlPoints;
        PoseSequence *pathPoints;

        // Controller State
        DirectController directController;
        int lookaheadPointIndex = 0;                           // Closest path index to the lookahead
        int controlPointIndex = 0;                             // Current control index of the event
        bool skipCheckpoints = false;                          // Whether the controller can skip checkpoints
        double lookaheadDistance = DEFAULT_LOOKAHEAD_DISTANCE; // in
    };
}