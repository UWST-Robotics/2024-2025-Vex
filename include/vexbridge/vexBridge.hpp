#pragma once

#include <cstring>
#include "table/valueTable.hpp"
#include "table/labelTable.hpp"
#include "serial/serialWriter.hpp"
#include "serial/serialSocket.hpp"
#include "serial/drivers/usbSerialDriver.hpp"
#include "serial/drivers/vexSerialDriver.hpp"

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
         * @param port The port to connect to. `0` for USB, `1 - 21` for VEX V5 ports.
         */
        VEXBridge(uint8_t port)
        {
            if (port == 0)
                socket = std::make_unique<SerialSocket>(std::make_unique<USBSerialDriver>());
            else if (port >= 1 && port <= 21)
                socket = std::make_unique<SerialSocket>(std::make_unique<VEXSerialDriver>(port));
            else
                throw std::runtime_error("Invalid port number: " + std::to_string(port));
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
            if (!labelTable.contains(label))
                return defaultValue;
            int16_t id = labelTable.get(label);

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
            if (!labelTable.contains(label))
            {
                auto id = labelTable.create(label);
                SerialWriter::assignLabel(id, label);
            }
            return labelTable.get(label);
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
            if (!valueTable.isType<T>(id))
                throw std::runtime_error("Type mismatch for value " + std::to_string(id));

            // Check if the value exists
            if (!valueTable.contains(id))
                return defaultValue;

            // Get the value
            return valueTable.get<T>(id);
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
            if (valueTable.contains(id) &&
                valueTable.get<T>(id) == value)
                return;

            valueTable.set(id, value);

            // Send the serial packet
            if (std::is_same<T, bool>::value)
                SerialWriter::updateBoolean(id, value);
            else if (std::is_same<T, int>::value)
                SerialWriter::updateInt(id, value);
            else if (std::is_same<T, float>::value)
                SerialWriter::updateFloat(id, value);
            else if (std::is_same<T, double>::value)
                SerialWriter::updateDouble(id, value);
            // TODO: Add support for strings
            else
                throw std::runtime_error("Unsupported type to update: " + std::string(typeid(T).name()));
        }

    private:
        static ValueTable valueTable;
        static LabelTable labelTable;

        std::unique_ptr<SerialSocket> socket;
    };
}

// Initialize static members
ValueTable vexbridge::VEXBridge::valueTable;
LabelTable vexbridge::VEXBridge::labelTable;