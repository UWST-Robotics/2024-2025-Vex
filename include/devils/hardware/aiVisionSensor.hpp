#pragma once

#include <string>
#include <cstdint>
#include "structs/hardwareBase.hpp"
#include "structs/camera.h"
#include "../geometry/units.hpp"
#include "../sdkExtensions.h"

namespace devils
{
    /**
     * Direct implementation of the AI Vision Sensor.
     * This bypasses PROS and uses the VEX SDK directly.
     * As a result, mutexes are not used and the user must ensure thread safety.
     */
    class AIVisionSensor : private HardwareBase, public ICamera
    {
    public:
        // Thank you James Pearman for these measurements
        // https://www.vexforum.com/t/vision-sensor-fov-measurements/62397
        // TODO: Update me!
        static constexpr int VISION_WIDTH_PX = 0;    // px
        static constexpr int VISION_HEIGHT_PX = 0;   // px
        static constexpr int VISION_WIDTH_FOV = 61;  // degrees
        static constexpr int VISION_HEIGHT_FOV = 41; // degrees

        /**
         * Creates a new instance of the AI Vision Sensor.
         * @param name - The name of the sensor used for logging
         * @param port - The port of the sensor (1 - 21)
         */
        AIVisionSensor(std::string name, uint16_t port)
            : HardwareBase(name, "AIVision", port),
              port(port)
        {
        }

        /**
         * Gets the objects detected by the sensor.
         * @return The objects detected by the sensor
         */
        std::vector<ICamera::VisionObject> getObjectsInView() override
        {
            std::vector<ICamera::VisionObject> objects;
            int32_t count = getObjectCount();
            for (int32_t i = 0; i < count; i++)
            {
                auto object = getObject(i);
                objects.push_back(ICamera::VisionObject{
                    object.object.color.xoffset,
                    object.object.color.yoffset,
                    object.object.color.width,
                    object.object.color.height});
            }
            return objects;
        }

        const ICamera::Parameters getParameters() override
        {
            return ICamera::Parameters{
                VISION_WIDTH_PX,
                VISION_HEIGHT_PX,
                Units::degToRad(VISION_WIDTH_FOV),
                Units::degToRad(VISION_HEIGHT_FOV)};
        }

        /**
         * Gets the object at the specified index.
         * Use `getObjectCount()` to get the number of objects detected.
         * @param index - The index of the object to get
         * @return The object at the specified index
         */
        V5_DeviceAiVisionObject getObject(int32_t index)
        {
            V5_DeviceAiVisionObject object;
            // vexDeviceAiVisionObjectGet(port, index, &object);
            return object;
        }

        /**
         * Gets the number of objects detected by the sensor.
         * @return The number of objects detected
         */
        int32_t getObjectCount()
        {
            return 0;
            // return vexDeviceAiVisionObjectCountGet(port);
        }

    protected:
        void serialize() override
        {
        }

    private:
        uint16_t port;
    };
}