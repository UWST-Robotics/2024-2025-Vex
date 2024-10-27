#pragma once
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace devils
{
    /**
     * A class containing a variety of unit conversion utilities.
     */
    class Units
    {
    public:
        /**
         * Converts inches to meters.
         * @param inches The inches to convert.
         * @return The inches in meters.
         */
        static double inToMeters(double inches)
        {
            return inches * 0.0254;
        }

        /**
         * Converts meters to inches.
         * @param meters The meters to convert.
         * @return The meters in inches.
         */
        static double metersToIn(double meters)
        {
            return meters / 0.0254;
        }

        /**
         * Compares the difference of two radian angles.
         * @param radiansA The first angle in radians.
         * @param radiansB The second angle in radians.
         * @return The difference between the two angles, relative to `radiansB`. Can be [-PI, PI]
         */
        static double diffRad(double radiansA, double radiansB)
        {
            return std::atan2(std::sin(radiansA - radiansB), std::cos(radiansA - radiansB));
        }

        /**
         * Converts degrees to radians.
         * @param degrees The degrees to convert.
         * @return The degrees in radians.
         */
        static double degToRad(double degrees)
        {
            return degrees * (M_PI / 180.0);
        }

        /**
         * Converts centidegrees to radians.
         * @param centidegrees The centidegrees to convert.
         * @return The centidegrees in radians.
         */
        static double centidegToRad(double centidegrees)
        {
            return centidegrees * (M_PI / 18000.0);
        }

        /**
         * Modulus function that also works with negative numbers.
         * @param a The number to mod.
         * @param b The modulus.
         * @return The result of the modulus. Always positive.
         */
        static double mod(double a, double b)
        {
            return a - std::floor(a / b) * b;
        }

        /**
         * Converts radians to degrees.
         * @param radians The radians to convert.
         */
        static double radToDeg(double radians)
        {
            return radians * (180.0 / M_PI);
        }

        /**
         * Normalizes an angle in radians to be between 0 and 2 * PI.
         * @param radians The angle in radians to normalize.
         * @return The normalized angle in radians.
         */
        static double normalizeRadians(double radians)
        {
            return mod(radians, 2 * M_PI);
        }

        /**
         * Converts inches to a string.
         * @param inches The inches to convert.
         * @return The string representation of the inches and feet
         */
        static std::string inToString(int inches)
        {
            // Handle Negative
            bool negative = inches < 0;
            if (negative)
                inches = -inches;

            // Split Inches and Feet
            int feet = inches / 12;
            inches = inches % 12;

            // Create String
            std::stringstream stream;
            if (negative)
                stream << "-";
            if (feet > 0)
                stream << feet << "' ";
            stream << inches << "\"";

            return stream.str();
        }
    };
}