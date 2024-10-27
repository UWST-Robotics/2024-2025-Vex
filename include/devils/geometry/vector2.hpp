#pragma once
#include <string>
#include <cmath>

namespace devils
{
    /**
     * Represents a 3D vector.
     */
    struct Vector2
    {
        /// @brief The x position.
        double x = 0;
        /// @brief The y position.
        double y = 0;

        /**
         * Constructs a 3D vector with all values set to 0
         */
        Vector2() : x(0), y(0) {}

        /**
         * Constructs a vector with the given x, and y
         * @param x The x position
         * @param y The y position
         */
        Vector2(double x, double y) : x(x), y(y) {}

        /**
         * Copy constructor
         * @param other The other vector
         */
        Vector2(const Vector2 &other) : x(other.x), y(other.y) {}

        /**
         * Constructs a vector by copying another vector
         * @param other The other vector
         */
        Vector2 operator=(const Vector2 &other)
        {
            x = other.x;
            y = other.y;
            return *this;
        }

        /**
         * Adds two vectors together
         * @param other The other vector
         * @return The sum of the two vectors
         */
        Vector2 operator+(const Vector2 &other)
        {
            return {x + other.x, y + other.y};
        }

        /**
         * Subtracts one vector from another
         * @param other The other vector
         * @return The difference of the two vectors
         */
        Vector2 operator-(const Vector2 &other)
        {
            return {x - other.x, y - other.y};
        }

        /**
         * Multiplies a vector by a scalar
         * @param scalar The scalar to multiply by
         * @return The vector multiplied by the scalar
         */
        Vector2 operator*(const double &scalar)
        {
            return {x * scalar, y * scalar};
        }

        /**
         * Compares two vectors for equality
         * @param other The other vector
         * @return True if the vectors are equal, false otherwise
         */
        bool operator==(const Vector2 &other)
        {
            return x == other.x && y == other.y;
        }

        /**
         * Compares two vectors for inequality
         * @param other The other vector
         * @return True if the vectors are not equal, false otherwise
         */
        bool operator!=(const Vector2 &other)
        {
            return !(*this == other);
        }

        /**
         * Calculates the dot product of two vectors
         * @param other The other vector
         * @return The dot product of the two vectors
         */
        double dot(const Vector2 &other)
        {
            return x * other.x + y * other.y;
        }

        /**
         * Calculates the distance between two vectors
         * @param other The other vector
         * @return The distance between the two vectors
         */
        double distanceTo(const Vector2 &other)
        {
            return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
        }

        /**
         * Calculates the magnitude of the vector
         * @return The magnitude of the vector
         */
        double magnitude()
        {
            return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
        }

        /**
         * Normalizes the vector
         * @return The normalized vector
         */
        Vector2 normalize()
        {
            double mag = magnitude();
            return {x / mag, y / mag};
        }

        /**
         * Prints the vector to a string
         * @return The vector as a string
         */
        const std::string toString()
        {
            return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
        }
    };
}