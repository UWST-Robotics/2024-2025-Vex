#pragma once
#include "vexbridge/vexBridge.h"
#include "structs/camera.h"

namespace devils
{
    /**
     * Represents an offboard Raspberry Pi running OpenCV.
     * Data is sent over the network using VEX Bridge.
     */
    class DevilCV : public ICamera
    {
    public:
        static constexpr double VISION_WIDTH_FOV = 55; // degrees

        /**
         * Creates a DevilCV object.
         * @param name The name of the camera
         */
        DevilCV(std::string name)
        {
            setCameraName(name);
        }

        bool hasTargets() override
        {
            return hasTarget->get();
        }

        ICamera::VisionObject getClosestTarget() override
        {
            return ICamera::VisionObject{targetX->get(), targetY->get()};
        }

        /**
         * Sets the camera name for the VEX Bridge.
         * @param name The name of the camera
         */
        void setCameraName(std::string name)
        {
            targetX = std::make_unique<VBValue<float>>("vision/color/" + name + "/center_x", 0.0f);
            targetY = std::make_unique<VBValue<float>>("vision/color/" + name + "/center_y", 0.0f);
            hasTarget = std::make_unique<VBValue<bool>>("vision/color/" + name + "/has_target", false);
        }

    private:
        std::unique_ptr<VBValue<float>> targetX = std::make_unique<VBValue<float>>(nullptr);
        std::unique_ptr<VBValue<float>> targetY = std::make_unique<VBValue<float>>(nullptr);
        std::unique_ptr<VBValue<bool>> hasTarget = std::make_unique<VBValue<bool>>(nullptr);
    };
}