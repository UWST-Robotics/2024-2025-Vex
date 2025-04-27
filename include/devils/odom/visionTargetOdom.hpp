#pragma once

#include "odomSource.hpp"
#include "../hardware/structs/camera.h"
#include "../geometry/units.hpp"
#include <stdexcept>
#include <memory>

namespace devils
{
    /**
     * Offsets the odometry of the robot based on the vision target.
     * Corrects for slight deviations in the robot's position by using the vision target as a reference.
     */
    class VisionTargetOdom : public OdomSource
    {
    public:
        /**
         * Constructs a new vision target with the given expected pose
         * @param baseOdom The base odometry source to use
         * @param camera The camera to use
         * @param cameraFOV The camera's field of view in degrees
         * @param expectedPose The expected pose of the vision target.
         */
        VisionTargetOdom(
            OdomSource &baseOdom,
            std::shared_ptr<ICamera> camera,
            double cameraFOV,
            double latency = 0.0,
            Pose expectedPose = Pose(0, 0, 0))
            : baseOdom(baseOdom),
              delayedOdom(baseOdom, latency),
              camera(camera),
              expectedPose(expectedPose),
              cameraFOV(Units::degToRad(cameraFOV))
        {
            if (!camera)
                throw std::invalid_argument("Camera cannot be null");
        }

        /**
         * Converts a vision target in the camera's view to an absolute pose.
         * @param target The vision target to convert
         * @return The estimated pose of the target
         */
        Pose visionTargetToPose(const ICamera::VisionObject &target)
        {
            // Get pose of the robot at the time of the target's detection (delayed odometry)
            Pose robotPose = delayedOdom.getPose();

            // Calculate the angle of the target in degrees
            double targetAngle = 0.5 * cameraFOV * target.x + robotPose.rotation;

            // Calculate the offset from the camera to the target
            // TODO: Estimate the distance of the target using the vision target's size
            Vector2 targetOffset = Vector2(
                TARGET_DISTANCE_TO_CAMERA * std::cos(targetAngle),
                TARGET_DISTANCE_TO_CAMERA * std::sin(targetAngle));

            // Append the offset to the robot's pose
            return Pose(
                robotPose.x + targetOffset.x,
                robotPose.y + targetOffset.y,
                0); // <-- Must be 0 to avoid appending rotation to the odometry
        }

        Pose getPose() override
        {
            // Get the current pose of the robot
            Pose robotPose = baseOdom.getPose();

            // Fallback to base odometry if no targets are found
            if (!camera->hasTargets())
                return robotPose + lastOffset;

            // Get the pose of the closest target
            auto visionTarget = camera->getClosestTarget();
            auto targetPose = visionTargetToPose(visionTarget);

            // Check if the target is too far away
            if (targetPose.distanceTo(expectedPose) > MAX_DISTANCE)
                return robotPose + lastOffset;

            // Calculate the offset from the expected pose to the target pose
            Pose offset = expectedPose - targetPose;
            lastOffset = offset;

            // Correct the robot's pose using the offset
            return robotPose + offset;
        }

        void setPose(Pose pose) override
        {
            baseOdom.setPose(pose);
        }

        PoseVelocity getVelocity() override
        {
            return baseOdom.getVelocity();
        }

        /**
         * Change the expected pose of the vision target.
         * Used if using the same vision target in multiple locations.
         * @param pose The new expected pose of the robot
         */
        void setExpectedPose(Pose pose)
        {
            expectedPose = pose;
        }

    private:
        /// @brief The target's maximum distance from the expected pose to be considered valid
        static constexpr double MAX_DISTANCE = 6.0;

        /// @brief A vision target's *assumed* distance to the camera in inches
        static constexpr double TARGET_DISTANCE_TO_CAMERA = 8.0;

        OdomSource &baseOdom;
        DelayedOdom delayedOdom;
        std::shared_ptr<ICamera> camera = nullptr;
        double cameraFOV; // radians

        /// @brief The last offset applied to the robot's pose
        /// @details This is used to correct the robot's pose when no targets are found
        Pose lastOffset = Pose(0, 0, 0);

        /// @brief The expected pose of the vision target
        Pose expectedPose;
    };
}