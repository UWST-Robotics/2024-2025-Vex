#pragma once
#include "smartMotorGroup.hpp"
#include "../utils/math.hpp"

namespace devils
{
    /**
     * Represents a conveyor chain loop with a specified length.
     * This class is used to calculate the position of the conveyor chain loop in chain links.
     */
    class ChainLoop
    {
    public:
        /**
         * Represents a conveyor chain loop with a specified length.
         * @param motorGroup The SmartMotorGroup that represents the motors in the chain loop.
         * @param sprocketTeeth The amount of teeth on the sprocket.
         * @param length The amount of chain in the loop.
         * @param startingOffset The offset of the first motor in the loop.
         */
        ChainLoop(SmartMotorGroup &motorGroup,
                  int sprocketTeeth,
                  int length,
                  double startingOffset = 0)
            : motorGroup(motorGroup),
              sprocketTeeth(sprocketTeeth),
              length(length),
              startingOffset(startingOffset)
        {
        }

        /**
         * Gets the position of the loop in chain links.
         */
        double getPosition()
        {
            // Get the encoder position
            double encoderPosition = motorGroup.getPosition();

            // Get the revolutions of the sprocket
            double revolutions = encoderPosition / ENCODER_TICKS_PER_REVOLUTION;

            // Get the final position in chain links
            double position = revolutions * sprocketTeeth + startingOffset;

            // Modulo the position to keep it within the length of the chain loop
            return Math::signedMod(position, length);
        }

        /**
         * Gets the distance to a target position in chain links.
         * @param targetPosition The target position in chain links.
         * @return The distance to the target position in chain links.
         */
        double getDistanceToPosition(double targetPosition)
        {
            // Get the current position of the loop
            double currentPosition = getPosition();

            // Calculate the distance to the target position
            double distance = Math::signedMod(targetPosition - currentPosition, length);

            // Return the distance to the target position
            return distance;
        }

    private:
        /// @brief The amount of encoder ticks per revolution of the conveyor motors.
        static constexpr double ENCODER_TICKS_PER_REVOLUTION = 300.0;

        SmartMotorGroup &motorGroup;
        int sprocketTeeth = 0;
        int length = 0;
        double startingOffset = 0;
    };
}