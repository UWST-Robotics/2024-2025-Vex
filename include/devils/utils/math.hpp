#pragma once

#include <cmath>
#include <algorithm>

namespace devils
{
    struct Math
    {
        /**
         * Linearly interpolates along a trapezoidal profile.
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
            double minSpeed,
            double maxSpeed)
        {
            double accelPercent = std::clamp(distanceToStart / accelDistance, -1.0, 1.0); // Percent of distance to start
            double decelPercent = std::clamp(distanceToEnd / decelDistance, -1.0, 1.0);   // Percent of distance to target
            double speedPercent = std::min(fabs(accelPercent), fabs(decelPercent));       // Use the smaller of the two
            double speed = std::lerp(minSpeed, maxSpeed, speedPercent);                   // Lerp between min and max speed
            speed *= std::copysign(1.0, distanceToEnd);                                   // Apply direction
            return speed;
        }

        /**
         * Modulus function that works with negative numbers.
         * @param a The dividend.
         * @param b The divisor.
         * @return The remainder.
         */
        static double signedMod(double a, double b)
        {
            return a - b * std::floor(a / b);
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
    };
}