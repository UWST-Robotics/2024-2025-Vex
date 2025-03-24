#pragma once
#include "chassisBase.hpp"
#include "../hardware/smartMotorGroup.hpp"
#include <vector>
#include <iostream>
#include "../utils/logger.hpp"
#include "../odom/odomSource.hpp"
#include "../odom/poseVelocityCalculator.hpp"
#include "../utils/runnable.hpp"

namespace devils
{
    /**
     * Represents a chassis that maintains a virtual position and orientation.
     * This is useful for testing autonomous routines without a physical robot.
     * Can be used as an OdomSource.
     */
    class DummyChassis : public ChassisBase, public OdomSource, public Runnable, public PoseVelocityCalculator
    {
    public:
        DummyChassis()
        {
            // Run on startup
            runAsync();
        }

        void stop() override
        {
            ChassisBase::stop();
        }

        void move(double forward, double turn, double strafe = 0) override
        {
            forward = std::clamp(forward, -1.0, 1.0);
            turn = std::clamp(turn, -1.0, 1.0);
            strafe = std::clamp(strafe, -1.0, 1.0);

            lastForward = forward;
            lastTurn = turn;
            lastStrafe = strafe;
        }

        void onUpdate() override
        {
            // TODO: Multiply acceleration by delta time

            // Calculate Acceleration
            currentAcceleration.x += (cos(currentPose.rotation) * lastForward + sin(currentPose.rotation) * lastStrafe) * TRANSLATION_ACCEL;
            currentAcceleration.y += (sin(currentPose.rotation) * lastForward + cos(currentPose.rotation) * lastStrafe) * TRANSLATION_ACCEL;
            currentAcceleration.rotation += lastTurn * ROTATION_ACCEL;
            currentAcceleration = currentAcceleration * (1 - DRAG);

            // Update Pose
            currentPose = currentPose + currentAcceleration;
            currentPose.rotation = std::fmod(currentPose.rotation, 2 * M_PI);

            // Update Velocity
            updateVelocity(currentPose);
        }

        void setPose(Pose &pose) override
        {
            currentPose = pose;
        }

        Pose &getPose() override
        {
            return currentPose;
        }

        PoseVelocity &getVelocity() override
        {
            return PoseVelocityCalculator::getVelocity();
        }

    private:
        static constexpr double TRANSLATION_ACCEL = 0.8; // in/s^2
        static constexpr double ROTATION_ACCEL = 0.1;    // rad/s^2
        static constexpr double DRAG = 0.35;             // %

        double lastForward = 0;
        double lastTurn = 0;
        double lastStrafe = 0;
        Pose currentAcceleration = Pose();
        Pose currentPose = Pose();
    };
}