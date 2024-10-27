#pragma once
#include <string>
#include <cmath>
#include "vector2.hpp"

namespace devils
{
    /**
     * Represents a pose in 2D space.
     */
    struct Pose : public Vector2
    {
        /// @brief The x position of the robot in inches
        double &x = Vector2::x;

        /// @brief The y position of the robot in inches
        double &y = Vector2::y;

        /// @brief The rotation of the robot in radians
        double rotation = 0;

        /**
         * Constructs a pose with all values set to 0
         */
        Pose() : rotation(0) {}

        /**
         * Constructs a pose with the given x and y
         * @param x The x position of the robot in inches
         * @param y The y position of the robot in inches
         */
        Pose(double x, double y) : Vector2(x, y), rotation(0) {}

        /**
         * Constructs a pose with the given x, y, and rotation
         * @param x The x position of the robot in inches
         * @param y The y position of the robot in inches
         * @param rotation The rotation of the robot in radians
         */
        Pose(double x, double y, double rotation) : Vector2(x, y), rotation(rotation) {}

        /**
         * Constructs a pose by copying another vector. Sets rotation to 0.
         * @param other The other vector
         */
        Pose(const Vector2 &other) : Vector2(other.x, other.y), rotation(0) {}

        /**
         * Copy constructor
         * @param other The other pose
         */
        Pose(const Pose &other) : Vector2(other.x, other.y), rotation(other.rotation) {}

        /**
         * Constructs a pose by copying another pose
         * @param other The other pose
         */
        Pose operator=(const Pose &other)
        {
            x = other.x;
            y = other.y;
            rotation = other.rotation;
            return *this;
        }

        /**
         * Adds two poses together
         * @param other The other pose
         * @return The sum of the two poses
         */
        Pose operator+(const Pose &other)
        {
            return {x + other.x, y + other.y, rotation + other.rotation};
        }

        /**
         * Subtracts one pose from another
         * @param other The other pose
         * @return The difference of the two poses
         */
        Pose operator-(const Pose &other)
        {
            return {x - other.x, y - other.y, rotation - other.rotation};
        }

        /**
         * Multiplies a pose by a scalar
         * @param scalar The scalar to multiply by
         * @return The pose multiplied by the scalar
         */
        Pose operator*(const double &scalar)
        {
            return {x * scalar, y * scalar, rotation * scalar};
        }

        /**
         * Compares two poses for equality
         * @param other The other pose
         * @return True if the poses are equal, false otherwise
         */
        bool operator==(const Pose &other)
        {
            return x == other.x && y == other.y && rotation == other.rotation;
        }

        /**
         * Compares two poses for inequality
         * @param other The other pose
         * @return True if the poses are not equal, false otherwise
         */
        bool operator!=(const Pose &other)
        {
            return !(*this == other);
        }

        /**
         * Normalizes the pose
         * @return The normalized pose
         */
        Pose normalize()
        {
            double mag = magnitude();
            return {x / mag, y / mag, rotation};
        }

        /**
         * Prints the pose to a string
         * @return The pose as a string
         */
        const std::string toString()
        {
            return "Pose(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(rotation) + ")";
        }
    };

    /**
     * A list of poses to play in sequence
     */
    typedef std::vector<Pose> PoseSequence;
}