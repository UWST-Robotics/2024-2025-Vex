#pragma once

#include <unordered_map>
#include <string>

typedef std::unordered_map<std::string, std::string> NetworkTableCache;

namespace devils
{
    /**
     * Utility for transmitting key/value pairs over serial
     */
    class NetworkTables
    {
    public:
        // Delete Constructor
        NetworkTables() = delete;

        /**
         * Resets all values in the network table.
         * Should be called at the start of runtime.
         */
        static void reset()
        {
            // Clear the cache
            cache.clear();

            // Send the reset message over serial
            printf("__NTRESET__\n");
        }

        /**
         * Sends a heartbeat message to the network table.
         */
        static void sendHeartbeat()
        {
            // Send the heartbeat message over serial
            printf("__NTHEARTBEAT__\n");
        }

        /**
         * Sends a message to the to update a string value.
         * @param key The key to update including the full path. (e.g. "/devils/robot/position/x")
         * @param value The value to update.
         */
        static void updateValue(std::string key, std::string value)
        {
            // Check if the value has changed
            for (auto &pair : cache)
                if (pair.first == key && pair.second == value)
                    return;

            // Update the cache
            cache[key] = value;

            // Send the update over serial
            std::cout << "__NTUPDATE__ " << key << " " << value << std::endl;
            // printf("__NTUPDATE__ %s %s\n", key.c_str(), value.c_str());
        }

        /**
         * Sends a message to the to update a numeric value.
         * @param key The key to update including the full path. (e.g. "/devils/robot/position/x")
         * @param value The value to update.
         */
        static void updateDoubleValue(std::string key, double value)
        {
            updateValue(key, std::to_string(value));
        }

        /**
         * Sends a message to the to update a numeric value.
         * @param key The key to update including the full path. (e.g. "/devils/robot/position/x")
         * @param value The value to update.
         */
        static void updateIntValue(std::string key, int value)
        {
            updateValue(key, std::to_string(value));
        }

        /**
         * Sends a message to the to update a boolean value.
         * @param key The key to update including the full path. (e.g. "/devils/robot/position/x")
         * @param value The value to update.
         */
        static void updateBoolValue(std::string key, bool value)
        {
            updateValue(key, value ? "true" : "false");
        }

    private:
        static NetworkTableCache cache;
    };
}

// Initialize the cache
NetworkTableCache devils::NetworkTables::cache;