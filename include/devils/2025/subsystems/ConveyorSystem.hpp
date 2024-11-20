#pragma once

#include "../../devils.h"

namespace devils
{
    /**
     * Represents the conveyor belt system of the robot.
     */
    class ConveyorSystem
    {
    public:
        ConveyorSystem(std::initializer_list<int8_t> conveyorPorts, uint8_t grabberPort)
            : conveyorMotors("ConveyorMotors", conveyorPorts),
              grabberPneumatic("GrabberPneumatic", grabberPort)
        {
        }

        /**
         * Runs the conveyor system at a given voltage. Automatically stops the conveyor system if a ring is detected.
         * @param voltage The voltage to run the conveyor system at, from -1 to 1.
         */
        void runAutomatic()
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

            // Stop the conveyor system if a ring is detected
            // and we don't have a mogo grabbed
            if (isRingDetected && !isGrabbed)
            {
                conveyorMotors.stop();
                return;
            }

            // Run the conveyor system
            conveyorMotors.moveVoltage(isGrabbed ? MOGO_CONVEYOR_SPEED : NO_MOGO_CONVEYOR_SPEED);
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
         * Grabs a mogo with the grabber.
         */
        void grabGoal()
        {
            grabberPneumatic.extend();
        }

        /**
         * Releases a mogo from the grabber.
         */
        void releaseGoal()
        {
            grabberPneumatic.retract();
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
        static constexpr double NO_MOGO_CONVEYOR_SPEED = 0.5;
        static constexpr double MOGO_CONVEYOR_SPEED = 1.0;

        SmartMotorGroup conveyorMotors;
        ADIPneumatic grabberPneumatic;
        OpticalSensor *sensor = nullptr;
    };
}