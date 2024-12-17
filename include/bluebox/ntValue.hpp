#pragma once

#include "ntSerial.hpp"

uint16_t idCounter = 0;

namespace devils
{
    template <typename T>
    class NTValue
    {
    public:
        NTValue(const std::string path, const T defaultValue)
            : path(path),
              value(defaultValue)
        {
            // Label ID
            NTSerial *serialInstance = NTSerial::getInstance();
            if (serialInstance == nullptr)
                return;

            serialInstance->labelID(id, path.c_str());
        }

        /**
         * Sets the value of the NTValue.
         */
        void set(T value)
        {
            // Cache the value
            if (value == this->value)
                return;

            // Check if BlueBox is initialized
            NTSerial *serialInstance = NTSerial::getInstance();
            if (serialInstance == nullptr)
                return;

            // Transmit the value
            if constexpr (std::is_same<T, bool>::value)
                serialInstance->updateBoolean(id, value);
            else if constexpr (std::is_same<T, int>::value)
                serialInstance->updateInt(id, value);
            else if constexpr (std::is_same<T, double>::value)
                serialInstance->updateDouble(id, value);
            else if constexpr (std::is_same<T, std::string>::value)
                serialInstance->updateString(id, value.c_str());
            else
                Logger::error("NTValue: Unsupported type");
        }

        /**
         * Gets the value of the NTValue.
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