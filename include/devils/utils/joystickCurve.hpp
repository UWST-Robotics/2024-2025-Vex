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
         * Linearly interpolates a value with a deadzone
         * @param deadzone The deadzone of the joystick.
         * @param min The minimum value of the joystick.
         * @param max The maximum value of the joystick.
         * @param val The value of the joystick.
         * @return The interpolated value.
         */
        static double dlerp(double deadzone, double min, double max, double val)
        {
            if (val > -deadzone && val < deadzone)
                return 0;
            return lerp(min, max, val);
        }

        /**
         * Linearly interpolates a value.
         * @param a The minimum value.
         * @param b The maximum value.
         * @param t The value to interpolate (0 - 1).
         * @return The interpolated value.
         */
        static double lerp(double a, double b, double t)
        {
            if (t< 0)
                return ((1 + t) * a - t * b) * -1;
            return (1 - t) * a + t * b;
        }

        /**
         * Curves on an arbitrary power curve.
         * @param val The value to curve.
         * @param power The power to curve by.
         * @return The curved value.
         */
        static double pow(double val, double power)
        {
            return std::pow(std::abs(val), power) * (val < 0 ? -1 : 1);
        }

        /**
         * Curves on an arbitrary curve with a deadzone.
         * @param val The value to curve.
         * @param power The power to curve by.
         * @param deadzone The range of which the joystick is considered 0.
         * @return The curved value.
         */
        static double curve(double val, double power, double deadzone)
        {
            if (val > -deadzone && val < deadzone)
                return 0;
            return pow(val, power);
        }

        /**
         * Curves on a square curve.
         * @param val The value to curve.
         * @return The curved value.
         */
        static double square(double val)
        {
            return val * val * (val < 0 ? -1 : 1);
        }

        /**
         * Curves on a cubic curve.
         * @param val The value to curve.
         * @return The curved value.
         */
        static double cubic(double val)
        {
            return val * val * val;
        }

    private:
        // Prevent instantiation
        JoystickCurve() = delete;
    };
}