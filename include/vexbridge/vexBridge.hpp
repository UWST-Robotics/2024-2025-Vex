#pragma once

#include <cstring>
#include "table/ValueTable.hpp"
#include "table/LabelTable.hpp"
#include "serial/drivers/usbSerialDriver.hpp"
#include "serial/serialSocket.hpp"
#include "serial/serialWriter.hpp"

using namespace vexbridge::table;
using namespace vexbridge::serial;

namespace vexbridge
{
    /**
     * Interface for interacting with the VEXBridge.
     */
    class VEXBridge
    {
    public:
        /**
         * Opens a new socket connection to the VEXBridge.
         * Once instantiated, all calls to `VEXBridge` can be made statically.
         */
        VEXBridge()
            : socket(std::make_unique<SerialSocket>(std::make_unique<USBSerialDriver>()))
        {
        }

        /**
         * Retrieves a value from VEXBridge.
         * @param label The label of the value.
         * @param defaultValue The default value to return if the value does not exist.
         * @return The value of the value or the default value if the value does not exist.
         */
        template <typename T>
        static T get(const std::string label, const T defaultValue)
        {
            // Get the ID of the value
            if (!LabelTable::contains(label))
                return defaultValue;
            int16_t id = LabelTable::get(label);

            // Get the value
            return getByID<T>(id, defaultValue);
        }

        /**
         * Updates a value to the VEXBridge.
         * Sends a packet to the VEXBridge to update the value.
         * @param label The label of the value.
         * @param value The new value.
         */
        template <typename T>
        static void set(const std::string label, const T value)
        {
            setByID(getOrAssignID(label), value);
        }

        /**
         * Gets the ID of a label from the VEXBridge.
         * If the label does not exist, it will be assigned an ID.
         * @param label The label to get the ID of.
         * @return The ID of the label.
         */
        static uint16_t getOrAssignID(const std::string label)
        {
            if (!LabelTable::contains(label))
            {
                auto id = LabelTable::create(label);
                SerialWriter::assignLabel(id, label);
            }
            return LabelTable::get(label);
        }

        /**
         * Gets the value of an ID from the VEXBridge.
         * @param id The ID of the value.
         * @param defaultValue The default value to return if the value does not exist.
         * @return The value of the value or the default value if the value does not exist.
         */
        template <typename T>
        static T getByID(const uint16_t id, const T defaultValue)
        {
            // Check the type
            if (!ValueTable::isType<T>(id))
                throw std::runtime_error("Type mismatch for value " + std::to_string(id));

            // Check if the value exists
            if (!ValueTable::contains(id))
                return defaultValue;

            // Get the value
            return ValueTable::get<T>(id);
        }

        /**
         * Updates a value to the VEXBridge.
         * Sends a packet to the VEXBridge to update the value.
         * @param id The ID of the value.
         * @param value The new value.
         */
        template <typename T>
        static void setByID(const uint16_t id, const T value)
        {
            if (ValueTable::contains(id) &&
                ValueTable::get<T>(id) == value)
                return;

            ValueTable::set(id, value);
            updateValue<T>(id, value);
        }

    protected:
        template <typename T>
        static void updateValue(const uint16_t id, const T value)
        {
            // Uses template specialization to determine the correct packet type
            throw std::runtime_error("Unsupported type to update: " + std::string(typeid(T).name()));
        }

    private:
        std::unique_ptr<SerialSocket> socket;
    };
}

// Append template specializations for each type
template <>
void vexbridge::VEXBridge::updateValue<bool>(const uint16_t id, const bool value)
{
    SerialWriter::updateBool(id, value);
}
template <>
void vexbridge::VEXBridge::updateValue<int>(const uint16_t id, const int value)
{
    SerialWriter::updateInt(id, value);
}
template <>
void vexbridge::VEXBridge::updateValue<float>(const uint16_t id, const float value)
{
    SerialWriter::updateFloat(id, value);
}
template <>
void vexbridge::VEXBridge::updateValue<double>(const uint16_t id, const double value)
{
    SerialWriter::updateDouble(id, value);
}
template <>
void vexbridge::VEXBridge::updateValue<std::string>(const uint16_t id, const std::string value)
{
    SerialWriter::updateString(id, value);
}
template <>
void vexbridge::VEXBridge::updateValue<std::vector<bool>>(const uint16_t id, const std::vector<bool> value)
{
    SerialWriter::updateBoolArray(id, value);
}
template <>
void vexbridge::VEXBridge::updateValue<std::vector<int>>(const uint16_t id, const std::vector<int> value)
{
    SerialWriter::updateIntArray(id, value);
}
template <>
void vexbridge::VEXBridge::updateValue<std::vector<float>>(const uint16_t id, const std::vector<float> value)
{
    SerialWriter::updateFloatArray(id, value);
}
template <>
void vexbridge::VEXBridge::updateValue<std::vector<double>>(const uint16_t id, const std::vector<double> value)
{
    SerialWriter::updateDoubleArray(id, value);
}