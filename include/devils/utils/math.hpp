#pragma once

#include <cmath>
#include <algorithm>

namespace devils
{
    struct Math
    {
        /**
         * Interpolates along a trapezoidal profile. Used to minimize acceleration.
         * @param distanceToStart The distance to the start of the profile.
         * @param distanceToEnd The distance to the end of the profile.
         * @param accelDistance The distance to start accelerating.
         * @param decelDistance The distance to start decelerating.
         * @param minSpeed The minimum speed of the profile.
         * @param maxSpeed The maximum speed of the profile.
         * @return The speed at the given distance.
         */
        static double trapezoidProfile(
            double distanceToStart,
            double distanceToEnd,
            double accelDistance,
            double decelDistance,
            double minAccelSpeed,
            double minDecelSpeed,
            double maxSpeed)
        {
            double accelPercent = std::fabs(std::clamp(distanceToStart / accelDistance, -1.0, 1.0)); // Percent of distance to start
            double decelPercent = std::fabs(std::clamp(distanceToEnd / decelDistance, -1.0, 1.0));   // Percent of distance to target

            bool isAccel = accelPercent < decelPercent; // Check if we are accelerating or decelerating

            double speedPercent = isAccel ? accelPercent : decelPercent; // Use the smaller of the two
            double minSpeed = isAccel ? minAccelSpeed : minDecelSpeed;   // Use the correct min speed

            double speed = std::lerp(minSpeed, maxSpeed, speedPercent); // Lerp between min and max speed
            speed *= std::copysign(1.0, distanceToEnd);                 // Apply direction
            return speed;
        }

        /**
         * Interpolates along a sigmoid (S-Curve) profile. Used to minimize jerk.
         * @param distanceToStart The distance to the start of the profile.
         * @param distanceToEnd The distance to the end of the profile.
         * @param accelDistance The distance to start accelerating.
         * @param decelDistance The distance to start decelerating.
         * @param minSpeed The minimum speed of the profile.
         * @param maxSpeed The maximum speed of the profile.
         * @param kCurve The strength of the curve. Must be greater than 1.
         * @return The speed at the given distance.
         */
        static double sigmoidProfile(
            double distanceToStart,
            double distanceToEnd,
            double accelDistance,
            double decelDistance,
            double minSpeed,
            double maxSpeed,
            double kCurve = 2.0)
        {
            double accelPercent = std::clamp(distanceToStart / accelDistance, -1.0, 1.0); // Percent of distance to start
            double decelPercent = std::clamp(distanceToEnd / decelDistance, -1.0, 1.0);   // Percent of distance to target
            double speedPercent = std::min(fabs(accelPercent), fabs(decelPercent));       // Use the smaller of the two

            double curvedSpeed = sigmoidCurve(speedPercent, kCurve);   // Curve the speed
            double speed = std::lerp(minSpeed, maxSpeed, curvedSpeed); // Lerp between min and max speed
            speed *= std::copysign(1.0, distanceToEnd);                // Apply direction
            return speed;
        }

        /**
         * Returns the minimum of two values by magnitude.
         * @param valueA The first value.
         * @param valueB The second value.
         * @return The value with the minimum magnitude.
         */
        static double minMagnitude(
            double valueA,
            double valueB)
        {
            if (std::abs(valueA) < std::abs(valueB))
                return valueA;
            else
                return valueB;
        }

        /**
         * Returns the maximum of two values by magnitude.
         * @param valueA The first value.
         * @param valueB The second value.
         * @return The value with the maximum magnitude.
         */
        static double maxMagnitude(
            double valueA,
            double valueB)
        {
            if (std::abs(valueA) > std::abs(valueB))
                return valueA;
            else
                return valueB;
        }

        /**
         * Interpolates a value along a sigmoid curve.
         * @param x The input value.
         * @param kCurve The curve factor. Must be greater than 1.
         * @return The value along the curve.
         */
        static double sigmoidCurve(
            double x,
            double kCurve)
        {
            // Avoid division by zero
            if (x == 1)
                return 1;

            // Calculate the curve
            double hyperbolic = std::pow(x / (1 - x), kCurve);
            return 1 - 1 / (1 + hyperbolic);
        }

        /**
         * Modulus function that works with negative numbers.
         * For example, -1 % 3 = 2 and -1 % -3 = 1.
         * @param a The dividend.
         * @param b The divisor.
         * @return The remainder.
         */
        static double signedMod(double a, double b)
        {
            return a - b * std::floor(a / b);
        }

        /**
         * Clamps a value between a minimum and maximum.
         * Allows for a deadband around zero such that negative values are clamped to -min and positive values are clamped to +min.
         * @param value The value to clamp.
         * @param min The minimum value.
         * @param max The maximum value.
         * @return The clamped value.
         */
        static double deadbandClamp(
            const double value,
            const double min,
            const double max)
        {
            if (value >= 0)
                return std::clamp(value, min, max);
            else
                return std::clamp(value, -max, -min);
        }

        /**
         * Calculates the difference between two radian angles.
         * @param a The first angle in radians.
         * @param b The second angle in radians.
         * @return The minimum difference between the two angles in radians.
         */
        static double angleDiff(double a, double b)
        {
            double dist = a - b;
            dist = signedMod(dist + M_PI, 2 * M_PI) - M_PI;
            return dist;
        }

        /**
         * Calculates the difference between two degree angles.
         * @param a The first angle in degrees.
         * @param b The second angle in degrees.
         * @return The minimum difference between the two angles in degrees.
         */
        static double angleDiffDeg(double a, double b)
        {
            double dist = a - b;
            dist = signedMod(dist + 180, 360) - 180;
            return dist;
        }

        /**
         * Calculates the end velocity over a specified distance given initial velocity and acceleration.
         * @param initialVelocity The initial velocity in units per second.
         * @param acceleration Constant acceleration in units per second squared.
         * @param distance The distance in units.
         */
        static double velocityOverDist(
            double initialVelocity,
            double acceleration,
            double distance)
        {
            return std::sqrt(
                initialVelocity * initialVelocity +
                2 * acceleration * distance);
        }

        /**
         * Calculates the acceleration over a specified distance given initial and final velocities.
         * @param initialVelocity The initial velocity in units per second.
         * @param finalVelocity The final velocity in units per second.
         * @param distance The distance in units.
         */
        static double accelOverDist(
            double initialVelocity,
            double finalVelocity,
            double distance)
        {
            return (finalVelocity * finalVelocity - initialVelocity * initialVelocity) / (2 * distance);
        }

        /**
         * Calculates the distance between b and c along the line defined by a and b.
         * @param a The first point on the line.
         * @param b The second point on the line. Also the point to measure from.
         * @param c The point to measure to.
         * @return The distance between b and c along the line defined by a and b.
         */
        static double distanceOnLine(
            Vector2 a,
            Vector2 b,
            Vector2 c)
        {
            double numerator = (c.x - a.x) * (b.x - a.x) + (c.y - a.y) * (b.y - a.y);
            double denominator = (b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y);
            double t = numerator / denominator;

            double x = (b.x - ((1 - t) * a.x + t * b.x));
            double y = (b.y - ((1 - t) * a.y + t * b.y));
            double distance = std::sqrt(x * x + y * y);

            return distance;
        }
    };
}