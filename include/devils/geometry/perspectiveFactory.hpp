#pragma once

#include "perspective.hpp"
#include "../hardware/visionSensor.hpp"

namespace devils
{
    class PerspectiveFactory
    {
    public:
        /**
         * Builds a new perspective object based on the legacy V5 vision sensor's calibration.
         * @param cameraHeight The height of the camera above the ground in inches.
         * @param cameraPitch The pitch of the camera in degrees.
         * @return The perspective object.
         */
        static Perspective buildLegacyVisionSensor(double cameraHeight, double cameraPitch)
        {
            return Perspective(
                VisionSensor::VISION_HEIGHT_FOV,
                NEAR_CLIP,
                FAR_CLIP,
                VisionSensor::VISION_WIDTH_PX,
                VisionSensor::VISION_HEIGHT_PX,
                cameraHeight,
                cameraPitch);
        }

    private:
        PerspectiveFactory() = delete;

        static constexpr double NEAR_CLIP = 1.0;
        static constexpr double FAR_CLIP = 1000.0;
    };
}