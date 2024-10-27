#pragma once
#include "../geometry/vector2.hpp"

namespace devils
{
    /**
     * Represents an object detected by a vision sensor.
     */
    struct VisionObject : public Vector2
    {
        /**
         * Creates a new instance of VisionObject.
         */
        VisionObject(double x, double y, double area)
            : Vector2(x, y), area(area) {}

        /**
         * Copy constructor
         * @param other The other VisionObject
         */
        VisionObject(const VisionObject &other)
            : Vector2(other.x, other.y), area(other.area) {}

        /// @brief The area of the object in square pixels.
        double area = 0;
    };
}