#pragma once
#include "pros/motors.hpp"
#include "pros/vision.hpp"
#include "pros/error.h"
#include "../utils/logger.hpp"
#include "structs/hardwareBase.hpp"
#include "structs/camera.h"
#include "../geometry/units.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a VEX legacy vision sensor object.
     */
    class LegacyVisionSensor : private HardwareBase, public ICamera
    {
    public:
        // Thank you James Pearman for these measurements
        // https://www.vexforum.com/t/vision-sensor-fov-measurements/62397
        static constexpr int VISION_WIDTH_PX = VISION_FOV_WIDTH;   // px
        static constexpr int VISION_HEIGHT_PX = VISION_FOV_HEIGHT; // px
        static constexpr int VISION_WIDTH_FOV = 61;                // degrees
        static constexpr int VISION_HEIGHT_FOV = 41;               // degrees

        /**
         * Creates a vision sensor object.
         * @param name The name of the motor (for logging purposes)
         * @param port The port of the motor (from 1 to 21)
         */
        LegacyVisionSensor(std::string name, uint8_t port)
            : HardwareBase(name, "VisionSensor", port),
              sensor(port, pros::E_VISION_ZERO_CENTER)
        {
            if (errno != 0)
                reportFault("Invalid port");
        }

        /**
         * Sets the vision sensor's LED color.
         * Overrides the default LED behavior.
         * @param color The color to set the LED to
         */
        void setLEDColor(int32_t color)
        {
            auto status = sensor.set_led(color);
            if (status == PROS_ERR)
                reportFault("Failed to set LED color");
        }

        /**
         * Resets the vision sensor's LED color to the default behavior.
         */
        void resetLEDColor()
        {
            auto status = sensor.clear_led();
            if (status == PROS_ERR)
                reportFault("Failed to reset LED color");
        }

        bool hasTargets() override
        {
            int32_t objectCount = sensor.get_object_count();
            return objectCount > 0;
        }

        ICamera::VisionObject getClosestTarget() override
        {
            // Get the biggest object in view
            auto object = sensor.get_by_size(0);

            // Return the object as a VisionObject
            return ICamera::VisionObject{
                (double)object.x_middle_coord / VISION_WIDTH_PX,
                (double)object.y_middle_coord / VISION_HEIGHT_PX};
        }

    private:
        pros::Vision sensor;
    };
}