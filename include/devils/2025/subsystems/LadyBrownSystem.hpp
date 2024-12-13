#pragma once

#include "devils/devils.h"
#include "ConveyorSystem.hpp"

namespace devils
{
    class LadyBrownSystem : public Runnable
    {
    public:
        LadyBrownSystem(
            SmartMotorGroup &motors,
            RotationSensor &sensor,
            ConveyorSystem &conveyor)
            : motors(motors),
              sensor(sensor),
              conveyor(conveyor)
        {
        }

        void onUpdate() override
        {
            idle();
        }

        void setPosition(double targetPosition)
        {
            double currentPosition = Units::radToDeg(sensor.getAngle());
            double error = targetPosition - currentPosition;

            double speed = Math::trapezoidProfile(
                1,
                error,
                1,
                DECEL_DISTANCE,
                0,
                0,
                MAX_SPEED);

            motors.moveVoltage(speed);
        }

        void raise()
        {
            setPosition(RAISED_POSITION);
            conveyor.setLadyBrownState(false, true);
        }

        void lower()
        {
            setPosition(LOWERED_POSITION);
            conveyor.setLadyBrownState(true, false);
        }

        void idle()
        {
            setPosition(IDLE_POSITION);
            conveyor.setLadyBrownState(false, false);
        }

    private:
        // Constants
        static constexpr double LOWERED_POSITION = 0;  // deg
        static constexpr double IDLE_POSITION = 60;    // deg
        static constexpr double RAISED_POSITION = 180; // deg

        static constexpr double DECEL_DISTANCE = 90; // deg
        static constexpr double MAX_SPEED = 0.6;     // %

        // State
        PIDController pid = PIDController(0.01, 0.0, 0.3);

        // Hardware
        SmartMotorGroup &motors;
        RotationSensor &sensor;
        ConveyorSystem &conveyor;
    };
}