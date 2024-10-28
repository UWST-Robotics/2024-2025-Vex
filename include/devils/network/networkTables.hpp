#pragma once
#include <map>
#include <string>

typedef std::map<std::string, std::string> NetworkTableCache;

namespace devils
{
    /**
     * Utility for transmitting key/value pairs over serial
     */
    class NetworkTables
    {
    public:
        /**
         * Resets all values in the network table.
         * Should be called at the start of runtime.
         */
        static void Reset()
        {
            // Clear the cache
            GetCache().clear();

            // Send the reset message over serial
            printf("RESET\n");
        }

        /**
         * Logs a message to the network table log.
         * @param message The message to log.
         */
        static void LogMessage(std::string message)
        {
            // Send the log message over serial
            printf("LOG %s\n", message.c_str());
        }

        /**
         * Sends a message to the to update a value.
         * @param key The key to update including the full path. (e.g. "/devils/robot/position/x")
         * @param value The value to update.
         */
        static void UpdateValue(std::string key, std::string value)
        {
            // Check if the value has changed
            NetworkTableCache &networkTableCache = GetCache();
            if (networkTableCache[key] == value)
                return;

            // Update the cache
            networkTableCache[key] = value;

            // Send the update over serial
            printf("UPDATE %s %s\n", key.c_str(), value.c_str());
        }

        /**
         * Gets the network table key of a hardware device.
         * @param portType The type of the port (e.g. "adi", "vex", etc.)
         * @param port The port index of the hardware.
         * @return The network table key for the hardware.
         */
        static std::string GetHardwareKey(std::string portType, int8_t port)
        {
            return "_hardware/" + portType + std::to_string((uint8_t)port);
        }
        /**
         * Gets the network table cache.
         * @return The network table cache.
         */
        static NetworkTableCache &GetCache()
        {
            static NetworkTableCache networkTableCache;
            return networkTableCache;
        }
    };
}