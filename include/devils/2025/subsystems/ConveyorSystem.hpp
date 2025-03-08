#pragma once

#include "../../devils.h"

namespace devils
{
    /// @brief All possible ring types that can be detected by the optical sensor.
    enum class RingType
    {
        RED,
        BLUE,
        NONE // No ring detected or sensor is disabled
    };

    /**
     * Represents the conveyor belt system in the center of the robot.
     */
    class ConveyorSystem
    {
    public:
        ConveyorSystem(SmartMotorGroup &conveyorMotors)
            : conveyorMotors(conveyorMotors)
        {
        }

        /**
         * Gets the current ring type detected by the optical sensor.
         * @return The current ring type detected.
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

            // Calculate the closest to each hue
            double redDiff = std::abs(Math::angleDiffDeg(hue, RED_HUE));
            double blueDiff = std::abs(Math::angleDiffDeg(hue, BLUE_HUE));
            if (redDiff < blueDiff)
                return RingType::RED;
            else
                return RingType::BLUE;
        }

        /**
         * Runs the conveyor system at a given voltage. Automatically stops the conveyor system if a ring is detected.
         * Should be called every interval to update the state of the conveyor system.
         * @param targetSpeed The target speed to run the conveyor system at, from -1 to 1.
         */
        void moveAutomatic(double targetSpeed = 1.0)
        {
            // Current State
            RingType currentRing = getCurrentRing();
            bool isRingDetected = currentRing != RingType::NONE;

            // Check if the conveyor system has a mogo
            double maxSpeed = this->hasMogo ? MOGO_CONVEYOR_SPEED : NO_MOGO_CONVEYOR_SPEED;

            // Check if the arm is lowered
            if (isArmLowered)
                maxSpeed = 0;

            // Check if can't pickup a ring
            double stopPosition = std::fmod(getConveyorPosition() + HOOK_STOP_OFFSET, HOOK_INTERVAL);
            bool isInBottomPosition = std::abs(stopPosition) < HOOK_STOP_RANGE;
            bool shouldStop = (!canPickupRing && isInBottomPosition) || isStopped;
            if (shouldStop)
            {
                maxSpeed = 0;
                this->isStopped = true;
            }

            // Calculate the speed of the conveyor system
            double speed = std::min(maxSpeed, targetSpeed);
            bool isForwards = targetSpeed > 0;

            // Start the cooldown timer if another timer is finished
            if (stallTimer.finished())
                startCooldown(STALL_REVERSE_DURATION, STALL_SPEED);
            if (rejectionTimer.finished())
            {
                startCooldown(REJECTION_DURATION, POST_REJECTION_SPEED);
                rejectionTimer.stop();
            }

            // Start stalled timer if the conveyor is stalled
            bool isStalled = conveyorMotors.getCurrent() > STALL_CURRENT;
            bool shouldFixStall = isStalled && isForwards;
            if (shouldFixStall && !stallTimer.running())
                stallTimer.start();
            else if (!shouldFixStall)
                stallTimer.stop();

            // Blue Ring Detection
            bool isBlueRing = currentRing == this->sortRingColor;
            bool isSortingEnabled = this->sortRingColor != RingType::NONE;
            bool shouldReject = isBlueRing && isForwards && isSortingEnabled;
            if (shouldReject)
                isRejectingRing = true;

            // Blue Ring Delay
            double rejectPosition = std::fmod(getConveyorPosition() + REJECTION_OFFSET, HOOK_INTERVAL);
            bool isInRejectionPosition = std::abs(rejectPosition) < HOOK_REJECTION_RANGE;
            if (isRejectingRing && isInRejectionPosition)
            {
                startCooldown(REJECTION_DURATION, POST_REJECTION_SPEED);
                isRejectingRing = false;
            }

            // Run the conveyor system on cooldown mode
            if (cooldownTimer.running() && isForwards)
            {
                conveyorMotors.moveVoltage(cooldownSpeed);
                return;
            }

            // Blue Ring Rejection
            if (isRejectingRing)
            {
                conveyorMotors.moveVoltage(PRE_REJECTION_SPEED);
                return;
            }

            // Prevent rings from being pushed out when we don't have a mogo
            if (isRingDetected &&
                !this->hasMogo &&
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
         * @param sensor The sensor to use to detect rings.
         */
        void useSensor(OpticalSensor *sensor)
        {
            this->sensor = sensor;
            sensor->setLEDBrightness(100);
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
            cooldownTimer.setDuration(duration);
            cooldownTimer.start();
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
         * Sets whether the conveyor system should sort rings by color.
         * @param ringColor Color of rings to sort out or NONE to disable sorting.
         */
        void setRingSorting(RingType ringColor)
        {
            this->sortRingColor = ringColor;
        }

        /**
         * Sets whether a mogo is currently grabbed.
         * @param hasMogo True if a mogo is grabbed, false otherwise.
         */
        void setMogoGrabbed(bool hasMogo)
        {
            // Update the mogo actuation time
            bool didChange = hasMogo != this->hasMogo;
            if (didChange)
                startCooldown(MOGO_ACTUATION_DELAY);

            this->hasMogo = hasMogo;
        }

        /**
         * Sets whether the arm is lowered.
         * @param isArmLowered True if the arm is lowered, false otherwise.
         */
        void setArmLowered(bool isArmLowered)
        {
            this->isArmLowered = isArmLowered;
        }

        /**
         * Enables the conveyor system to pick up rings.
         * @param canPickupRing True if the conveyor system can pick up a ring, false otherwise.
         */
        void setPickupRing(bool canPickupRing)
        {
            this->canPickupRing = canPickupRing;
            if (canPickupRing)
                this->isStopped = false;
        }

    private:
        //      SENSOR OPTIONS

        /// @brief The optical sensor threshold to detect a ring.
        static constexpr double PROXIMITY_THRESHOLD = 0.4;

        //      MOGO ACTUATION OPTIONS

        /// @brief The speed of the conveyor system when a mogo is not grabbed. Slower to prevent rings from overshooting the optical sensor.
        static constexpr double NO_MOGO_CONVEYOR_SPEED = 0.5;

        /// @brief The speed of the conveyor system when a mogo is grabbed. Faster to push rings into the mogo.
        static constexpr double MOGO_CONVEYOR_SPEED = 0.8;

        /// @brief The duration to stop the conveyor system after pneumatic actuation. Prevents rings from pushing onto the mogo before its ready.
        static constexpr double MOGO_ACTUATION_DELAY = 600;

        //      ANTI-STALL OPTIONS

        /// @brief The minimum duration a stall must occur.
        static constexpr double STALL_MIN_DURATION = 300;

        /// @brief The current threshold to detect a stall (in mA).
        static constexpr double STALL_CURRENT = 1800;

        /// @brief The duration to reverse the conveyor system when stalled.
        static constexpr double STALL_REVERSE_DURATION = 200;

        /// @brief The speed to reverse the conveyor system while stalled.
        static constexpr double STALL_SPEED = -0.8;

        //      RING DETECTION OPTIONS

        /// @brief The hue of the red ring in degrees.
        static constexpr double RED_HUE = 0;

        /// @brief The hue of the blue ring in degrees.
        static constexpr double BLUE_HUE = 100;

        /// @brief The duration to stop the conveyor system when rejecting a blue ring.
        static constexpr double REJECTION_DURATION = 300;

        /// @brief The speed to run the conveyor system before rejecting a blue ring.
        static constexpr double PRE_REJECTION_SPEED = 1.0;

        /// @brief The speed to run the conveyor system after rejecting a blue ring.
        static constexpr double POST_REJECTION_SPEED = -0.5;

        /// @brief The max range of the hook to reject a blue ring.
        static constexpr double HOOK_REJECTION_RANGE = 3;

        /// @brief The offset of the conveyor chain to reject a blue ring in teeth.
        static constexpr double REJECTION_OFFSET = -42;

        //     STOP OPTIONS

        /// @brief The offset of the conveyor chain to reject a blue ring.
        static constexpr double HOOK_STOP_OFFSET = 30;

        /// @brief The max range of the hook to stop if can't pick up a ring.
        static constexpr double HOOK_STOP_RANGE = 2;

        //      ENCODER OPTIONS

        /// @brief The amount of encoder ticks per revolution of the conveyor motors.
        static constexpr double ENCODER_TICKS_PER_REVOLUTION = 300.0;

        /// @brief The amount of teeth on the sprocket.
        static constexpr int SPROCKET_TEETH = 12;

        /// @brief The length of the conveyor chain in teeth.
        static constexpr double CONVEYOR_LENGTH = 82;

        /// @brief The distance between each hook in teeth.
        static constexpr double HOOK_INTERVAL = 31;

        // State
        bool hasMogo = false;
        bool isArmLowered = false;
        bool isRejectingRing = false;
        bool canPickupRing = false;
        bool isStopped = false;
        double cooldownSpeed = 0;
        RingType sortRingColor = RingType::NONE;

        // Timers
        Timer cooldownTimer = Timer(0);
        Timer stallTimer = Timer(STALL_MIN_DURATION);
        Timer rejectionTimer = Timer(REJECTION_DURATION);

        // Hardware
        SmartMotorGroup &conveyorMotors;
        OpticalSensor *sensor = nullptr;
    };
}