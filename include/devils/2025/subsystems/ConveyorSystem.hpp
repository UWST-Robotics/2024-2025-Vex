#pragma once

#include "../../devils.h"

namespace devils
{
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
         * Runs the conveyor system at a given voltage. Automatically stops the conveyor system if a ring is detected.
         * @param voltage The voltage to run the conveyor system at, from -1 to 1.
         */
        void moveAutomatic(double targetSpeed = 1.0)
        {
            // Check if a ring is detected and if the grabber is extended
            bool isRingDetected = true;
            if (sensor != nullptr)
            {
                double proximity = sensor->getProximity();
                isRingDetected = proximity > PROXIMITY_THRESHOLD;
                NetworkTables::updateBoolValue("RingDetected", isRingDetected);
                NetworkTables::updateDoubleValue("Proximity", proximity);
            }
            bool isGrabbed = isGoalGrabbed();
            bool isForwards = targetSpeed > 0;

            // Stop the conveyor system if a ring is detected
            // and we don't have a mogo grabbed
            if (isRingDetected && !isGrabbed && isForwards)
            {
                conveyorMotors.stop();
                return;
            }

            // Stop the conveyor system if we are actuating the mogo
            bool isActuating = pros::millis() - mogoActuationTime < MOGO_ACTUATION_DELAY;
            if (isActuating && isForwards)
            {
                conveyorMotors.stop();
                return;
            }

            // Run the conveyor system
            double speed = isGrabbed ? MOGO_CONVEYOR_SPEED : NO_MOGO_CONVEYOR_SPEED;
            speed = std::min(speed, targetSpeed);
            conveyorMotors.moveVoltage(speed);
        }

        /**
         * Forces the conveyor system to move at a given voltage, regardless of whether a ring is detected.
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
         * Sets whether a mogo is currently grabbed.
         * @param isGrabbed True if a mogo is grabbed, false otherwise.
         */
        void setGoalGrabbed(bool isGrabbed)
        {
            grabberPneumatic.setExtended(isGrabbed);
            mogoActuationTime = pros::millis();
        }

        /**
         * Returns whether a mogo is currently grabbed.
         * @return True if a mogo is grabbed, false otherwise.
         */
        bool isGoalGrabbed()
        {
            return grabberPneumatic.getExtended();
        }

    private:
        static constexpr double PROXIMITY_THRESHOLD = 0.3;
        static constexpr double NO_MOGO_CONVEYOR_SPEED = 0.6;
        static constexpr double MOGO_CONVEYOR_SPEED = 1.0;
        static constexpr double MOGO_ACTUATION_DELAY = 300;

        int mogoActuationTime = 0;

        SmartMotorGroup &conveyorMotors;
        ADIPneumatic &grabberPneumatic;
        OpticalSensor *sensor = nullptr;
    };
}