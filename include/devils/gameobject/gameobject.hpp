#pragma once
#include "../geometry/pose.hpp"

namespace devils
{
    /**
     * Represents a game object on the field
     * \deprecated This class is not used in the final implementation
     */
    struct GameObject : public Pose
    {
        double firstSeenTime = -1;
        double lastSeenTime = -1;

        /**
         * Creates a new game object with a given pose
         * @param p The pose of the game object
         */
        GameObject(Pose p) : Pose(p)
        {
            firstSeenTime = pros::millis();
            lastSeenTime = pros::millis();
        }

        /**
         * Creates a new game object with a given pose
         * @param x The x position of the game object in inches
         * @param y The y position of the game object in inches
         * @param theta The angle of the game object in radians
         */
        GameObject(double x, double y, double theta) : Pose(x, y, theta)
        {
            firstSeenTime = pros::millis();
            lastSeenTime = pros::millis();
        }

        /**
         * Creates a new game object with a given pose.
         * Assumes the object was first seen at the given time.
         * @param other The game object to copy
         */
        GameObject(GameObject &other) : Pose(other)
        {
            firstSeenTime = other.firstSeenTime;
            lastSeenTime = other.lastSeenTime;
        }

        /**
         * Gets the age of the game object in milliseconds
         * @return The age of the game object in milliseconds
         */
        double getAge()
        {
            return pros::millis() - firstSeenTime;
        }

        /**
         * Renews the last seen time of the game object
         */
        void renew()
        {
            lastSeenTime = pros::millis();
        }

        /**
         * Gets the time since the game object was last seen
         * @return The time since the game object was last seen
         */
        double getTimeSinceLastSeen()
        {
            return pros::millis() - lastSeenTime;
        }
    };
}