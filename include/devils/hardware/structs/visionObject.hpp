#pragma once
#include "../../geometry/vector2.hpp"

namespace devils
{
    /**
     * Represents an object detected by a vision sensor.
     */
    struct VisionObject : public Vector2
    {
        /// @brief The area of the object in square pixels.
        double area = 0;
    };
}