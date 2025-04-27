#pragma once
#include "vexbridge/vexBridge.h"
#include "camera.h"

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

        bool hasTargets() override
        {
            return hasTarget.get();
        }

        ICamera::VisionObject getClosestTarget() override
        {
            return ICamera::VisionObject{targetX.get(), targetY.get()};
        }

    private:
        VBValue<float> targetX = VBValue<float>("vision/x", 0.0f);
        VBValue<float> targetY = VBValue<float>("vision/y", 0.0f);
        VBValue<bool> hasTarget = VBValue<bool>("vision/hasTarget", false);
    };
}