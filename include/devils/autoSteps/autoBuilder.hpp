#pragma once

#include "../geometry/units.hpp"
#include "../odom/odomSource.hpp"
#include "../chassis/chassisBase.hpp"
#include "./steps/autoJumpToStep.hpp"
#include "./steps/autoDriveStep.hpp"
#include "./steps/autoDriveToStep.hpp"
#include "./steps/autoDriveTimeStep.hpp"
#include "./steps/autoRotateToStep.hpp"
#include "./steps/autoTimeoutStep.hpp"
#include "./steps/autoPauseStep.hpp"
#include "./steps/autoPurePursuitStep.hpp"
#include "./steps/autoBoomerangStep.hpp"
#include "./transformer/poseTransform.h"

namespace devils
{
    class AutoBuilder
    {
    public:
        /**
         * A builder for creating closed-loop autonomous routines w/ a chassis and odometry source.
         * Used to quickly create autonomous routines with a fluent API.
         * @param chassis The chassis to control
         * @param odom The odometry source to use
         */
        AutoBuilder(
            ChassisBase &chassis,
            OdomSource &odom)
            : chassis(chassis),
              odom(odom)
        {
        }

        /**
         * Sets the pose of the robot
         * @param pose The pose to set the robot to
         * @returns A pointer to the created step
         */
        AutoStepPtr setPose(Pose pose)
        {
            // Set the current pose
            this->pose = pose;

            // Transform the pose
            Pose transformedPose = tryTransformPose(pose);

            // Return a new `AutoJumpToStep` with the given pose
            return std::make_unique<AutoJumpToStep>(odom, transformedPose);
        }

        /**
         * Sets the pose of the robot
         * @param x The x position to set the robot to in inches
         * @param y The y position to set the robot to in inches
         * @param rotation The rotation to set the robot to in degrees
         * @returns A pointer to the created step
         */
        AutoStepPtr setPose(double x, double y, double rotation)
        {
            // Create a new pose
            Pose pose = Pose(x, y, Units::degToRad(rotation));

            // Return a new `AutoJumpToStep` with the given pose
            return setPose(pose);
        }

        /**
         * Pauses the autonomous routine for a given duration
         * @param duration The duration to pause in milliseconds
         * @returns A pointer to the created step
         */
        AutoStepPtr pause(uint32_t duration)
        {
            return std::make_unique<AutoPauseStep>(duration);
        }

        /**
         * Drives to a given pose using boomerang control. See `AutoBoomerangStep` for more info.
         * @param x The x position to drive to in inches
         * @param y The y position to drive to in inches
         * @param rotation The rotation to drive to in degrees
         * @param timeout The timeout in milliseconds
         * @param options The options for the drive step
         * @returns A pointer to the created step
         */
        AutoStepPtr driveTo(
            double x,
            double y,
            double rotation,
            uint32_t timeout = 2000,
            AutoDriveToStep::Options options = AutoDriveToStep::Options::defaultOptions)
        {
            // Create a new pose
            Pose targetPose = Pose(x, y, Units::degToRad(rotation));

            // Return a new `AutoBoomerangStep` with the given pose
            return driveTo(targetPose, timeout, options);
        }

        /**
         * Drives to a given pose using boomerang control. See `AutoBoomerangStep` for more info.
         * @param pose The pose to drive to
         * @param timeout The timeout in milliseconds
         * @param options The options for the drive step
         * @returns A pointer to the created step
         */
        AutoStepPtr driveTo(
            Pose pose,
            uint32_t timeout = 2000,
            AutoDriveToStep::Options options = AutoDriveToStep::Options::defaultOptions)
        {
            // Set the current pose
            this->pose = pose;

            // Transform the pose
            Pose transformedPose = tryTransformPose(pose);

            // Return a new `AutoBoomerangStep` with the given pose
            return std::make_unique<AutoTimeoutStep>(std::make_unique<AutoBoomerangStep>(chassis, odom, transformedPose, options), timeout);
        }

        /**
         * Rotates the robot a given amount
         * @param distance The distance to rotate in degrees
         * @param timeout The timeout in milliseconds
         * @returns A pointer to the created step
         */
        AutoStepPtr rotate(
            double distance,
            uint32_t timeout = 2000,
            AutoRotateToStep::Options options = AutoRotateToStep::Options::defaultOptions)
        {
            // Calculate the new heading (in degrees)
            double newHeading = Units::radToDeg(pose.rotation) + distance;

            // Return a new `AutoRotateToStep` with the given heading
            return rotateTo(newHeading, timeout, options);
        }

        /**
         * Rotates the robot to a given heading
         * @param heading The heading to rotate to in degrees
         * @param timeout The timeout in milliseconds
         * @returns A pointer to the created step
         */
        AutoStepPtr rotateTo(
            double heading,
            uint32_t timeout = 2000,
            AutoRotateToStep::Options options = AutoRotateToStep::Options::defaultOptions)
        {
            // Convert & apply the heading to the current pose
            pose.rotation = Units::degToRad(heading);

            // Transform the pose
            Pose transformedPose = tryTransformPose(pose);

            // Return a new `AutoRotateToStep` with the given heading
            return std::make_unique<AutoTimeoutStep>(std::make_unique<AutoRotateToStep>(chassis, odom, transformedPose.rotation, options), timeout);
        }

        /**
         * Uses a pose transformation when building autonomous
         * @param transformer - The transformation to apply
         */
        void useTransformer(std::unique_ptr<PoseTransform> transformer)
        {
            this->transformer = std::move(transformer);
        }

    protected:
        /**
         * Tries to transform a pose using the assigned transformer, if any
         * @param pose - The pose to transform
         * @returns The transformed pose
         */
        Pose tryTransformPose(Pose pose)
        {
            // If a transformer is assigned, transform the pose
            if (transformer)
                return transformer->transform(pose);

            // Otherwise, return the original pose
            return pose;
        }

    private:
        /// @brief The current robot pose (pre-transform)
        Pose pose;

        /// @brief The active transformer used to transform poses
        std::unique_ptr<PoseTransform> transformer = nullptr;

        // Input references
        ChassisBase &chassis;
        OdomSource &odom;
    };
}