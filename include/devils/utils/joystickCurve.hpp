#pragma once
#include <string>
#include <unistd.h>

namespace devils
{
    /**
     * Represents various curve values to apply to joystick inputs.
     */
    struct JoystickCurve
    {
        /**
         * Applies a deadzone to a joystick input.
         * @param deadzone The deadzone of the joystick.
         * @param val The value of the joystick. Must be between -1 and 1.
         * @return Value between deadzone and 1. Negative values are preserved.
         */
        static double deadzone(
            double deadzone,
            double val)
        {
            // Check if the value is within the deadzone
            double absVal = std::abs(val);
            if (absVal < deadzone)
                return 0;
            if (absVal > 1)
                return std::copysign(1, val);

            // Correct the value so it starts at 0 instead of deadzone
            double correctedVal = (absVal - deadzone) / (1 - deadzone);

            // Apply the sign back to the value
            return std::copysign(correctedVal, val);
        }

        /**
         * Applies an exponential curve to a joystick input.
         * @param val The value of the joystick. Must be between -1 and 1.
         * @param power The power of the curve.
         */
        static double pow(double val, double power)
        {
            double absVal = std::abs(val);
            double exponent = std::pow(absVal, power);
            return std::copysign(exponent, val);
        }

        /**
         * Lerp that maintains the sign of the value.
         * @param min The minimum value.
         * @param max The maximum value.
         * @param val The value to lerp.
         * @return The lerped value. If val is negative, the result will be negative.
         */
        static double lerp(int min, int max, double val)
        {
            double absVal = std::abs(val);
            double lerpVal = std::lerp(min, max, absVal);
            return std::copysign(lerpVal, val);
        }

        /**
         * Applies a curve to a joystick input.
         * @param val The value of the joystick. Must be between -1 and 1.
         * @param power The power of the curve.
         * @param deadzone The deadzone of the joystick.
         * @param min The minimum output value
         * @param max The maximum output value
         */
        static double curve(
            double val,
            double power,
            double deadzone,
            double min = 0.0,
            double max = 1.0)
        {
            // Deadzone
            double newVal = JoystickCurve::deadzone(deadzone, val);

            // Avoid remaining calculations if the value is 0
            if (newVal == 0)
                return 0;

            // Apply curve
            newVal = JoystickCurve::pow(newVal, power);

            // Lerp
            return JoystickCurve::lerp(min, max, newVal);
        }

    private:
        // Prevent instantiation
        JoystickCurve() = delete;
    };
}