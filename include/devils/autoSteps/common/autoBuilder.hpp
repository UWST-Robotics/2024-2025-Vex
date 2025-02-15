#pragma once

#include "autoStepList.hpp"
#include "../../odom/odomSource.hpp"
#include "../../chassis/chassisBase.hpp"
#include "../steps/autoJumpToStep.hpp"
#include "../steps/autoDriveStep.hpp"
#include "../steps/autoDriveToStep.hpp"
#include "../steps/autoRotateToStep.hpp"
#include "../steps/autoTimeoutStep.hpp"
#include "../steps/autoPauseStep.hpp"
#include "../steps/autoPurePursuitStep.hpp"

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
        AutoBuilder(ChassisBase &chassis, OdomSource &odom) : chassis(chassis), odom(odom) {}

        /**
         * Adds a custom step to the auto routine
         * @param step The step to add
         */
        void addStep(AutoStep *step)
        {
            steps.push_back(step);
        }

        /**
         * Adds a custom step to the auto routine with a timeout
         * @param step The step to add
         * @param timeout The timeout in milliseconds
         */
        void addStep(AutoStep *step, uint32_t timeout)
        {
            steps.push_back(new AutoTimeoutStep(step, timeout));
        }

        /**
         * Converts the builder to an AutoStepList
         * @return An AutoStepList with the steps added to the builder
         */
        AutoStepList *build()
        {
            return new AutoStepList(steps);
        }

        /**
         * Sets the pose of the robot
         * @param pose The pose to set the robot to
         */
        void setPose(Pose pose)
        {
            this->pose = pose;
            steps.push_back(new AutoJumpToStep(odom, pose));
        }

        /**
         * Sets the pose of the robot
         * @param x The x position to set the robot to in inches
         * @param y The y position to set the robot to in inches
         * @param rotation The rotation to set the robot to in radians
         */
        void setPose(double x, double y, double rotation)
        {
            setPose(Pose(x, y, rotation));
        }

        /**
         * Pauses the autonomous routine for a given duration
         * @param duration The duration to pause in milliseconds
         */
        void pause(uint32_t duration)
        {
            steps.push_back(new AutoPauseStep(duration));
        }

        /**
         * Drives a given distance in inches
         * @param distance The distance to drive in inches
         * @param timeout The timeout in milliseconds
         */
        void drive(
            double distance,
            uint32_t timeout = 2000,
            AutoDriveToStep::Options options = AutoDriveToStep::Options::defaultOptions)
        {
            pose = Pose(
                pose.x + distance * std::cos(pose.rotation),
                pose.y + distance * std::sin(pose.rotation),
                pose.rotation);

            addStep(new AutoDriveToStep(chassis, odom, pose, options), timeout);
        }

        /**
         * Drives a given distance in inches relative to the current pose
         * @param distance The distance to drive in inches
         * @param timeout The timeout in milliseconds
         */
        void driveRelative(
            double distance,
            uint32_t timeout = 2000,
            AutoDriveToStep::Options options = AutoDriveToStep::Options::defaultOptions)
        {
            pose = Pose(
                pose.x + distance * std::cos(pose.rotation),
                pose.y + distance * std::sin(pose.rotation),
                pose.rotation);

            addStep(new AutoDriveStep(chassis, odom, distance, options), timeout);
        }

        /**
         * Drives along a spline curve to a given pose.
         * @param x The x position to drive to in inches
         * @param y The y position to drive to in inches
         * @param rotation The rotation to drive to in radians
         * @param timeout The timeout in milliseconds
         */
        void driveSpline(
            double x,
            double y,
            double rotation,
            double delta = 18.0,
            uint32_t timeout = 2000,
            AutoDriveToStep::Options options = AutoDriveToStep::Options::defaultOptions)
        {
            // Get start/end poses
            Pose prevPose = Pose(pose.x, pose.y, pose.rotation);
            pose = Pose(pose.x + x, pose.y + y, rotation);

            // Create a spline path
            SplinePath *path = SplinePath::makeArc(prevPose, pose, delta);
            addStep(new AutoPurePursuitStep(chassis, odom, path, options), timeout);
        }

        /**
         * Rotates the robot a given distance in radians
         * @param distance The distance to rotate in radians
         * @param timeout The timeout in milliseconds
         */
        void rotate(
            double distance,
            uint32_t timeout = 2000,
            AutoRotateToStep::Options options = AutoRotateToStep::Options::defaultOptions)
        {
            pose.rotation += distance;
            addStep(new AutoRotateToStep(chassis, odom, pose.rotation, options), timeout);
        }

        /**
         * Rotates the robot to a given heading in radians
         * @param heading The heading to rotate to in radians
         * @param timeout The timeout in milliseconds
         */
        void rotateTo(
            double heading,
            uint32_t timeout = 2000,
            AutoRotateToStep::Options options = AutoRotateToStep::Options::defaultOptions)
        {
            pose.rotation = heading;
            addStep(new AutoRotateToStep(chassis, odom, pose.rotation, options), timeout);
        }

    private:
        // State
        Pose pose;
        std::vector<AutoStep *> steps;

        // Params
        ChassisBase &chassis;
        OdomSource &odom;
    };
}