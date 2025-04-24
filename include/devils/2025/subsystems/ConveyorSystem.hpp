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
            : conveyorMotors(conveyorMotors),
              conveyorChain(conveyorMotors,
                            SPROCKET_TEETH,
                            CONVEYOR_LENGTH,
                            CONVEYOR_START_OFFSET)
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
            // DEBUG
            VEXBridge::set("position", conveyorChain.getPosition());

            // Current State
            RingType currentRing = getCurrentRing();
            bool isRingDetected = currentRing != RingType::NONE;

            // Check if the conveyor system has a mogo
            double maxSpeed = this->hasMogo ? MOGO_CONVEYOR_SPEED : NO_MOGO_CONVEYOR_SPEED;

            // Check if the arm is lowered
            if (isArmLowered)
                maxSpeed = 0;

            // Calculate the speed of the conveyor system
            double speed = std::min(maxSpeed, targetSpeed);
            bool isForwards = targetSpeed > 0;
            bool isStopped = targetSpeed == 0;

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

            // Start zoom timer if we have a ring
            bool shouldZoom = isRingDetected && isForwards && hasMogo;
            if (shouldZoom && !zoomTimer.running())
                zoomTimer.start();

            // Blue Ring Detection
            bool isBlueRing = currentRing == this->sortRingColor;
            bool isSortingEnabled = this->sortRingColor != RingType::NONE;
            bool shouldReject = isBlueRing && isForwards && isSortingEnabled;
            if (shouldReject)
            {
                isRejectingRing = true;
                printf("Rejecting blue ring\n");
                printf("Ring color: %f\n", sensor->getHue());
            }

            // Blue Ring Delay
            bool isInRejectionPosition = getHookAtPosition(REJECTION_POSITION);
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

            // Zoom Mode
            if (zoomTimer.running() && (isForwards || isStopped))
            {
                conveyorMotors.moveVoltage(ZOOM_SPEED);
                return;
            }

            // Blue Ring Rejection
            if (isRejectingRing)
            {
                conveyorMotors.moveVoltage(PRE_REJECTION_SPEED);
                return;
            }

            // Pause Mode
            bool nearPausePosition = getHookAtPosition(HOOK_PAUSE_POSITION, HOOK_MAX_DISTANCE * 3);
            bool atPausePosition = getHookAtPosition(HOOK_PAUSE_POSITION, HOOK_MAX_DISTANCE);
            if (atPausePosition && isPaused)
            {
                conveyorMotors.stop();
                return;
            }
            else if (nearPausePosition && isPaused)
            {
                conveyorMotors.moveVoltage(NEAR_PAUSE_SPEED);
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
         * Checks if a hook is at a given position.
         * @param position The position to check in chain links.
         * @param maxDistance The maximum distance to check from the position in chain links.
         * @return True if a hook is at the position, false otherwise.
         */
        bool getHookAtPosition(double position, double maxDistance = HOOK_MAX_DISTANCE)
        {
            // Get all the hook positions
            int hookCount = sizeof(HOOK_POSITIONS) / sizeof(HOOK_POSITIONS[0]);

            // Iterate through all the hook positions
            for (int i = 0; i < hookCount; i++)
            {
                // Get the current hook position
                double hookPosition = HOOK_POSITIONS[i];

                // Check if the hook is within range of the target position
                double distance = conveyorChain.getDistanceToPosition(hookPosition + position);
                if (distance < maxDistance)
                    return true;
            }

            // If no hook is within range, return false
            return false;
        }

        /**
         * Sets whether the conveyor system is paused.
         * Pauses the conveyor such that it is primed to pick up a ring when unpaused.
         * @param isPaused True if the conveyor system is paused, false otherwise.
         */
        void setPaused(bool isPaused = true)
        {
            this->isPaused = isPaused;
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

        //      ZOOM OPTIONS

        /// @brief The duration to stop the conveyor system when scoring.
        static constexpr double ZOOM_MIN_DURATION = 300;

        /// @brief The speed to run the conveyor system when scoring.
        static constexpr double ZOOM_SPEED = 0.8;

        //      RING DETECTION OPTIONS

        /// @brief The hue of the red ring in degrees.
        static constexpr double RED_HUE = 0;

        /// @brief The hue of the blue ring in degrees.
        static constexpr double BLUE_HUE = 170;

        /// @brief The duration to stop the conveyor system when rejecting a blue ring.
        static constexpr double REJECTION_DURATION = 300;

        /// @brief The speed to run the conveyor system before rejecting a blue ring.
        static constexpr double PRE_REJECTION_SPEED = 1.0;

        /// @brief The speed to run the conveyor system after rejecting a blue ring.
        static constexpr double POST_REJECTION_SPEED = -0.5;

        /// @brief The position of the conveyor chain to reject a blue ring in teeth.
        static constexpr double REJECTION_POSITION = 12;

        //     PAUSE OPTIONS

        /// @brief The position of the conveyor chain to pause at in teeth.
        static constexpr double HOOK_PAUSE_POSITION = 45;

        /// @brief The speed to run the conveyor system when nearing the pause position.
        static constexpr double NEAR_PAUSE_SPEED = 0.2;

        //      CHAIN OPTIONS

        /// @brief The amount of teeth on the sprocket.
        static constexpr int SPROCKET_TEETH = 12;

        /// @brief The length of the conveyor chain in teeth.
        static constexpr double CONVEYOR_LENGTH = 82;

        /// @brief The starting offset of the conveyor chain in teeth.
        static constexpr double CONVEYOR_START_OFFSET = 0;

        //      HOOK OPTIONS

        /// @brief The distance between each hook in teeth.
        static constexpr double HOOK_POSITIONS[] = {0, 51};

        /// @brief The max distance between a hook and a position to be considered in range.
        static constexpr double HOOK_MAX_DISTANCE = 4.0;

        // State
        bool hasMogo = false;
        bool isArmLowered = false;
        bool isRejectingRing = false;
        bool isPaused = false;
        double cooldownSpeed = 0;
        RingType sortRingColor = RingType::NONE;

        // Timers
        Timer cooldownTimer = Timer(0);
        Timer zoomTimer = Timer(ZOOM_MIN_DURATION);
        Timer stallTimer = Timer(STALL_MIN_DURATION);
        Timer rejectionTimer = Timer(REJECTION_DURATION);

        // Hardware
        SmartMotorGroup &conveyorMotors;
        OpticalSensor *sensor = nullptr;
        ChainLoop conveyorChain;
    };
}