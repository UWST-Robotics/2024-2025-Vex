#pragma once
#include "pose.hpp"
#include "units.hpp"
#include <cmath>

namespace devils
{
    struct Lerp
    {
        /**
         * Linearly interpolates a value from a to b.
         * @param a The minimum value.
         * @param b The maximum value.
         * @param t The ratio between a and b. Values between 0 and 1.
         * @return The interpolated value.
         */
        static double lerp(double a, double b, double t)
        {
            return a + (b - a) * t;
        }

        /**
         * Linearly interpolates a rotational value from a to b.
         * @param a The minimum value in radians.
         * @param b The maximum value in radians.
         * @param t The ratio between a and b. Values between 0 and 1.
         */
        static double rotation(double a, double b, double t)
        {
            double aMod = std::fmod(a, 2 * M_PI);
            double bMod = std::fmod(b, 2 * M_PI);
            double diff = std::abs(aMod - bMod);
            if (diff > M_PI)
            {
                if (aMod > bMod)
                {
                    return Units::normalizeRadians(lerp(aMod, bMod + 2 * M_PI, t));
                }
                else
                {
                    return Units::normalizeRadians(lerp(aMod + 2 * M_PI, bMod, t));
                }
            }
            return Units::normalizeRadians(lerp(aMod, bMod, t));
        }

        /**
         * Linearly interpolates a point from a to b.
         * @param a The minimum point.
         * @param b The maximum point.
         * @param t The ratio between a and b. Values between 0 and 1.
         * @return The interpolated point.
         */
        static Pose linearPoints(Pose &a, Pose &b, double t)
        {
            return Pose(
                lerp(a.x, b.x, t),
                lerp(a.y, b.y, t),
                rotation(a.rotation, b.rotation, t));
        }

        /**
         * Quadratically interpolates a point from a to b to c.
         * @param a The minimum point.
         * @param b The middle point.
         * @param c The maximum point.
         * @param t The ratio between a and c. Values between 0 and 1.
         * @return The interpolated point.
         */
        static Pose quadraticPoints(Pose &a, Pose &b, Pose &c, double t)
        {
            Pose ab = linearPoints(a, b, t);
            Pose bc = linearPoints(b, c, t);
            return linearPoints(ab, bc, t);
        }

        /**
         * Cubically interpolates a point from a to b to c to d.
         * @param a The minimum point.
         * @param b The middle point.
         * @param c The middle point.
         * @param d The maximum point.
         * @param t The ratio between a and d. Values between 0 and 1.
         * @return The interpolated point.
         */
        static Pose cubicPoints(Pose &a, Pose &b, Pose &c, Pose &d, double t)
        {
            Pose abc = quadraticPoints(a, b, c, t);
            Pose bcd = quadraticPoints(b, c, d, t);
            return linearPoints(abc, bcd, t);
        }
        
    private:
        Lerp() = delete;
    };
}