#pragma once

#include <vector>

namespace devils
{
    /**
     * Represents some kind of vision sensor that can measure objects in the field of view.
     */
    struct ICamera
    {
        /// @brief The physical parameters of the camera.
        struct Parameters
        {
            /// @brief The width of the image in pixels.
            double imageWidth;

            /// @brief The height of the image in pixels.
            double imageHeight;

            /// @brief The horizontal field of view of the camera in radians.
            double xFOV;

            /// @brief The vertical field of view of the camera in radians.
            double yFOV;
        };

        /// @brief Represents an object detected by the camera.
        struct VisionObject
        {
            /// @brief The x position of the object in pixels.
            int32_t x;

            /// @brief The y position of the object in pixels.
            int32_t y;

            /// @brief The width of the object in pixels.
            int32_t width;

            /// @brief The height of the object in pixels.
            int32_t height;
        };

        /**
         * Gets the camera's physical parameters.
         * @return The camera's physical parameters.
         */
        virtual const Parameters getParameters() = 0;

        /**
         * Gets a list of all objects in the camera's field of view.
         * @return A list of all objects in the camera's field of view.
         */
        virtual std::vector<VisionObject> getObjectsInView() = 0;
    };
}