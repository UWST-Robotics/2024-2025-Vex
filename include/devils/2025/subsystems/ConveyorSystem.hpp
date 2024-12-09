#pragma once

#include "../../devils.h"

namespace devils
{
    enum class RingType
    {
        RED,
        BLUE,
        NONE
    };

    /**
     * Represents the conveyor belt system of the robot.
     */
    class ConveyorSystem : public Runnable
    {
    public:
        ConveyorSystem(SmartMotorGroup &conveyorMotors,
                       ADIPneumatic &grabberPneumatic)
            : conveyorMotors(conveyorMotors),
              grabberPneumatic(grabberPneumatic)
        {
        }

        void onUpdate()
        {
            moveAutomatic();
        }

        /**
         * Gets the current ring type detected by the optical sensor.
         */
        RingType getCurrentRing()
        {
            // If no sensor is used, return no ring
            if (sensor == nullptr)
                return RingType::NONE;

            // Get the sensor data
            double proximity = sensor->getProximity();
            double hue = sensor->getHue();

            // If no ring is detected, return no ring
            if (proximity < PROXIMITY_THRESHOLD)
                return RingType::NONE;

            // Convert hue to radians
            double hueRadians = Units::degToRad(hue);
            double redRadians = Units::degToRad(RED_HUE);
            double blueRadians = Units::degToRad(BLUE_HUE);

            // Calculate the closest to each hue
            double redDiff = Math::angleDiff(hueRadians, redRadians);
            double blueDiff = Math::angleDiff(hueRadians, blueRadians);
            if (redDiff < blueDiff)
                return RingType::RED;
            else
                return RingType::BLUE;
        }

        /**
         * Runs the conveyor system at a given voltage. Automatically stops the conveyor system if a ring is detected.
         * @param voltage The voltage to run the conveyor system at, from -1 to 1.
         */
        void moveAutomatic(double targetSpeed = 1.0)
        {
            // Current State
            bool isGrabbed = goalGrabbed();
            RingType currentRing = getCurrentRing();
            bool isRingDetected = currentRing != RingType::NONE;

            // Calculate the speed of the conveyor system
            double maxSpeed = isGrabbed ? MOGO_CONVEYOR_SPEED : NO_MOGO_CONVEYOR_SPEED;
            double speed = std::min(maxSpeed, targetSpeed);
            bool isForwards = targetSpeed > 0;

            // Start the cooldown timer if another timer is finished
            if (stallTimer.finished())
                startCooldown(STALL_REVERSE_DURATION, STALL_SPEED);
            if (rejectionTimer.finished())
                startCooldown(REJECTION_DURATION, REJECTION_SPEED);

            // Start stalled timer if the conveyor is stalled
            bool isStalled = conveyorMotors.getCurrent() > STALL_CURRENT;
            bool shouldFixStall = isStalled && isForwards;
            if (shouldFixStall && !stallTimer.running())
                stallTimer.start(STALL_MIN_DURATION);
            else if (!shouldFixStall)
                stallTimer.stop();

            // Start the rejection timer if a blue ring is detected
            bool isBlueRing = currentRing == RingType::BLUE;
            bool shouldReject = isBlueRing && isForwards && enableSorting;
            if (shouldReject && !rejectionTimer.running())
                rejectionTimer.start(REJECTION_DELAY);
            else if (!shouldReject)
                rejectionTimer.stop();

            // Run the conveyor system on cooldown mode
            if (cooldownTimer.running() && isForwards)
            {
                conveyorMotors.moveVoltage(cooldownSpeed);
                return;
            }

            // Prevent rings from being pushed out when we don't have a mogo
            if (isRingDetected &&
                !isGrabbed &&
                isForwards &&
                !rejectionTimer.running())
            {
                conveyorMotors.stop();
                return;
            }

            // Otherwise, run at the target speed
            conveyorMotors.moveVoltage(speed);
        }

        /**
         * Forces the conveyor system to move at a given voltage, regardless of sensor input.
         * @param voltage The voltage to run the conveyor system at, from -1 to 1.
         */
        void forceMove(double voltage)
        {
            conveyorMotors.moveVoltage(voltage);
        }

        /**
         * Enables the use of a sensor to stop the conveyor system when a ring is detected.
         */
        void useSensor(OpticalSensor *sensor)
        {
            this->sensor = sensor;
        }

        /**
         * Disables the conveyor system for a given duration.
         * Only applies if `runAutomatic` is used.
         * @param duration The duration to disable the conveyor system for.
         * @param speed The speed to run the conveyor system at during the cooldown.
         */
        void startCooldown(double duration, double speed = 0)
        {
            cooldownSpeed = speed;
            cooldownTimer.start(duration);
        }

        /**
         * Sets whether a mogo is currently grabbed.
         * @param isGrabbed True if a mogo is grabbed, false otherwise.
         */
        void setGoalGrabbed(bool isGrabbed)
        {
            // Update the mogo actuation time
            bool didChange = isGrabbed != goalGrabbed();
            if (didChange)
                startCooldown(MOGO_ACTUATION_DELAY);

            // Actuate the grabber
            grabberPneumatic.setExtended(isGrabbed);
        }

        /**
         * Returns whether a mogo is currently grabbed.
         * @return True if a mogo is grabbed, false otherwise.
         */
        bool goalGrabbed()
        {
            return grabberPneumatic.getExtended();
        }

        /**
         * Gets the position of the conveyor system in teeth.
         * @return The position of the conveyor system in teeth.
         */
        double getConveyorPosition()
        {
            double revolutions = conveyorMotors.getPosition() / ENCODER_TICKS_PER_REVOLUTION;
            return Math::signedMod(revolutions * SPROCKET_TEETH, CONVEYOR_LENGTH);
        }

        /**
         * Enables or disables the sorting of blue rings.
         * If enabled, `runAutomatic` will reject blue rings.
         * @param enableSorting True to enable sorting, false to disable.
         * @note This is disabled by default.
         */
        void setSortingEnabled(bool enableSorting)
        {
            this->enableSorting = enableSorting;
        }

    private:
        //      SENSOR OPTIONS

        /// @brief The optical sensor threshold to detect a ring.
        static constexpr double PROXIMITY_THRESHOLD = 0.3;

        //      MOGO ACTUATION OPTIONS

        /// @brief The speed of the conveyor system when a mogo is not grabbed.
        static constexpr double NO_MOGO_CONVEYOR_SPEED = 0.7;

        /// @brief The speed of the conveyor system when a mogo is grabbed.
        static constexpr double MOGO_CONVEYOR_SPEED = 1.0;

        /// @brief The delay to delay the conveyor system after pneumatic actuation.
        static constexpr double MOGO_ACTUATION_DELAY = 600;

        //      ANTI-STALL OPTIONS

        /// @brief The minimum duration a stall must occur.
        static constexpr double STALL_MIN_DURATION = 300;

        /// @brief The current threshold to detect a stall (in mA).
        static constexpr double STALL_CURRENT = 2000;

        /// @brief The duration to reverse the conveyor system when stalled.
        static constexpr double STALL_REVERSE_DURATION = 700;

        /// @brief The speed to reverse the conveyor system when on cooldown.
        static constexpr double STALL_SPEED = -0.4;

        //      RING DETECTION OPTIONS

        /// @brief The hue of the red ring in degrees.
        static constexpr double RED_HUE = 0;

        /// @brief The hue of the blue ring in degrees.
        static constexpr double BLUE_HUE = 240;

        /// @brief The amount of time to wait before stopping the conveyor system after a blue ring is detected.
        static constexpr double REJECTION_DELAY = 300;

        /// @brief The duration to stop the conveyor system when rejecting a blue ring.
        static constexpr double REJECTION_DURATION = 200;

        /// @brief The speed to run the conveyor system at when rejecting a blue ring.
        static constexpr double REJECTION_SPEED = -0.1;

        //      ENCODER OPTIONS

        /// @brief The amount of encoder ticks per revolution of the conveyor motors.
        static constexpr double ENCODER_TICKS_PER_REVOLUTION = 300.0;

        /// @brief The amount of teeth on the sprocket.
        static constexpr int SPROCKET_TEETH = 36;

        /// @brief The length of the conveyor system in teeth.
        static constexpr int CONVEYOR_LENGTH = 15;

        // State
        bool enableSorting = false;
        double cooldownSpeed = 0;

        // Timers
        Timer cooldownTimer;
        Timer stallTimer;
        Timer rejectionTimer;

        // Hardware
        SmartMotorGroup &conveyorMotors;
        ADIPneumatic &grabberPneumatic;
        OpticalSensor *sensor = nullptr;
    };
}