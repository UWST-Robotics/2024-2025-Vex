#pragma once

#include <vector>

namespace devils
{
    /**
     * Represents some kind of vision sensor that can measure objects in the field of view.
     */
    struct ICamera
    {
        /// @brief Represents an object detected by the camera.
        struct VisionObject
        {
            /// @brief The x position of the object measured in the camera's frame. Represented as a percentage of the camera's width from -1 to 1.
            double x;

            /// @brief The y position of the object measured in the camera's frame. Represented as a percentage of the camera's height from -1 to 1.
            double y;
        };

        /**
         * Gets the closest vision object
         * @returns The closest vision object
         */
        virtual VisionObject getClosestTarget() = 0;

        /**
         * Returns true if the camera has detected any targets.
         * @return True if the camera has detected any targets, false otherwise.
         */
        virtual bool hasTargets() = 0;
    };
}