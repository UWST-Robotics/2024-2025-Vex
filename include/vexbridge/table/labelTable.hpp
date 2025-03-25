#pragma once

#include <unordered_map>
#include <cstdint>
#include <string>
#include "pros/rtos.hpp"

namespace vexbridge::table
{
    /**
     * Table to store labels and their corresponding IDs.
     */
    class LabelTable
    {
    public:
        /**
         * Gets the ID of a label.
         * @param label The label to search for.
         * @return The ID of the label or -1 if not found.
         */
        int32_t get(const std::string &label)
        {
            if (!contains(label))
                return -1;

            return labelToID.at(label);
        }

        /**
         * Checks if the table contains a label.
         * @param label The label to search for.
         * @return True if the label is contained, false otherwise.
         */
        bool contains(const std::string &label)
        {
            return labelToID.find(label) != labelToID.end();
        }

        /**
         * Assigns a new ID for a label.
         * @param label The label to assign an ID.
         * @return The new ID assigned.
         */
        uint16_t create(const std::string &label)
        {
            // Avoid race conditions
            mutex.take(0);

            // Create a new ID
            static uint16_t idCounter = ID_OFFSET;
            uint16_t id = idCounter++;

            // Assign the ID to the label
            labelToID[label] = id;

            // Release mutex and return the ID
            mutex.give();
            return id;
        }

    private:
        // The offset to start assigning IDs at
        static constexpr uint16_t ID_OFFSET = 0;

        pros::Mutex mutex;
        std::unordered_map<std::string, uint16_t> labelToID;
    };
}