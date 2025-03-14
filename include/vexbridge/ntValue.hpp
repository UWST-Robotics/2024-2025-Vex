#pragma once

#include "vexBridge.hpp"
#include "utils/ntLogger.hpp"

uint16_t idCounter = 0;

namespace vexbridge
{
    template <typename T>
    class NTValue
    {
    public:
        /**
         * Creates a new variable that is syncronized over serial.
         * Values can be written to using the `set` method or read using the `get` method.
         * @param path The path of the value in the network table. (e.g. "chassis/velocity")
         * @param defaultValue The default value of the variable.
         */
        NTValue(const std::string path, const T defaultValue)
            : path(path),
              value(defaultValue)
        {
            // Label ID
            VEXBridge *serialInstance = VEXBridge::getInstance();
            if (serialInstance == nullptr)
                return;

            serialInstance->labelID(id, path.c_str());

            // Update the value
            forceSet(defaultValue);
        }

        /**
         * Sets the value of the NTValue.
         * @param value The value to set.
         */
        void set(T value)
        {
            // Cache the value
            if (value == this->value)
                return;

            // Update the value
            forceSet(value);
        }

        void forceSet(T value)
        {
            // Set the value
            this->value = value;

            // Check if BlueBox is initialized
            VEXBridge *serialInstance = VEXBridge::getInstance();
            if (serialInstance == nullptr)
                return;

            // Transmit the value
            if constexpr (std::is_same<T, bool>::value)
                serialInstance->updateBoolean(id, value);
            else if constexpr (std::is_same<T, int>::value)
                serialInstance->updateInt(id, value);
            else if constexpr (std::is_same<T, float>::value)
                serialInstance->updateFloat(id, value);
            else if constexpr (std::is_same<T, double>::value)
                serialInstance->updateDouble(id, value);
            else if constexpr (std::is_same<T, std::string>::value)
                serialInstance->updateString(id, value.c_str());
            else
                NTLogger::logWarning("NTValue is using unsupported type " + std::string(typeid(T).name()));
        }

        /**
         * Gets the value of the NTValue.
         * @return The current value of the NTValue.
         */
        T get()
        {
            return value;
        }

    protected:
        const uint16_t id = idCounter++;
        const std::string path;
        T value;
    };
}