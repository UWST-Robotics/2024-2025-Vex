#pragma once

#include <cstdint>
#include "vexbridge.hpp"

namespace vexbridge
{
    /**
     * Value that is synchronized over VEXBridge.
     */
    template <typename T>
    class VBValue
    {
    public:
        /**
         * Creates a new value with a label and default value.
         * @param label The label of the value.
         * @param defaultValue The default value.
         */
        VBValue(const std::string label, const T defaultValue)
            : id(VEXBridge::getOrAssignID(label)),
              defaultValue(defaultValue)
        {
            // Set the default value
            set(defaultValue);
        }

        /**
         * Gets the value from VEXBridge.
         * @return The current value.
         */
        T get() const
        {
            return VEXBridge::getByID<T>(id, defaultValue);
        }

        /**
         * Sets the value to VEXBridge.
         * @param value The new value.
         */
        void set(T value) const
        {
            VEXBridge::setByID(id, value);
        }

    private:
        const uint16_t id;
        const T defaultValue;
    };
}