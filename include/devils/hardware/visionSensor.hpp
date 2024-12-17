#pragma once
#include "pros/motors.hpp"
#include "pros/vision.hpp"
#include "motor.hpp"
#include "../utils/logger.hpp"
#include "../geometry/perspective.hpp"
#include "structs/visionObject.hpp"
#include "hardwareBase.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a vision sensor object. All events are logged.
     */
    class VisionSensor : private HardwareBase
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
        VisionSensor(std::string name, uint8_t port)
            : HardwareBase(name, "VisionSensor", port),
              sensor(port, pros::E_VISION_ZERO_CENTER)
        {
            if (errno != 0)
                reportFault("Invalid port");
        }

        /**
         * Gets the angle of an object relative to the center of the vision sensor.
         * @param object The object to get the angle of
         * @return The angle of the object in radians
         */
        static double getAngle(VisionObject object)
        {
            return Units::degToRad((object.x * VISION_WIDTH_FOV) / VISION_WIDTH_PX);
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

    protected:
        void serialize() override
        {
            // TODO: Serialize Hardware
        }

    private:
        pros::Vision sensor;
    };
}