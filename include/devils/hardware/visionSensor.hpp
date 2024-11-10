#pragma once
#include "pros/motors.hpp"
#include "pros/vision.hpp"
#include "motor.hpp"
#include "../utils/logger.hpp"
#include "../utils/ramp.hpp"
#include "../geometry/perspective.hpp"
#include "../utils/visionObject.hpp"
#include "../network/networkObject.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a vision sensor object. All events are logged.
     */
    class VisionSensor : private INetworkObject
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
            : name(name),
              sensor(port, pros::E_VISION_ZERO_CENTER)
        {
            if (errno != 0 && LOGGING_ENABLED)
                Logger::error(name + ": port is invalid");
        }

        /**
         * Gets any objects detected by the vision sensor.
         * @return The objects detected by the vision sensor
         */
        std::vector<VisionObject> getObjects()
        {
            std::vector<VisionObject> objects = {};

            // Get object count
            int objectCount = sensor.get_object_count();
            if (objectCount == 0)
                return objects;

            // Handle Errors
            if (objectCount == PROS_ERR)
            {
                if (LOGGING_ENABLED)
                    Logger::error(name + ": failed to get object count");
                return objects;
            }

            // Get objects
            for (int i = 0; i < objectCount; i++)
            {
                // Get Object
                pros::vision_object_s_t object = sensor.get_by_size(i);
                if (object.signature == 0)
                    continue;

                // Filter out small objects
                if (object.width * object.height < MIN_OBJECT_AREA)
                    continue;

                // Create Display Object
                short x = object.x_middle_coord;
                short y = object.y_middle_coord;
                objects.push_back(VisionObject((double)x, (double)y, (double)(object.width * object.height)));
            }

            return objects;
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
            if (status == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": failed to set LED color");
        }

        /**
         * Resets the vision sensor's LED color to the default behavior.
         */
        void resetLEDColor()
        {
            auto status = sensor.clear_led();
            if (status == PROS_ERR && LOGGING_ENABLED)
                Logger::error(name + ": failed to reset LED color");
        }

        void serialize() override
        {
            // Get Prefix
            std::string networkTableKey = NetworkTables::GetHardwareKey("vex", sensor.get_port());

            // Update Network Table
            NetworkTables::UpdateValue(networkTableKey + "/name", name);
            NetworkTables::UpdateValue(networkTableKey + "/type", "VisionSensor");

            if (!sensor.is_installed())
                NetworkTables::UpdateValue(networkTableKey + "/faults", "Disconnected");
            else
                NetworkTables::UpdateValue(networkTableKey + "/faults", "");
        }

    private:
        static constexpr bool LOGGING_ENABLED = false;
        static constexpr double MIN_OBJECT_AREA = 3 * 3; // px^2

        std::string name;
        pros::Vision sensor;
    };
}