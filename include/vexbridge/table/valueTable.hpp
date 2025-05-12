#pragma once

#include <map>
#include <cstdint>
#include <any>
#include <stdexcept>

namespace vexbridge::table
{
    class ValueTable
    {
    public:
        /**
         * Sets a value within the table.
         * @param id The id of the value to set.
         * @param value The value to set.
         */
        template <typename T>
        static void set(const uint16_t id, const T value)
        {
            idToPath[id] = value;
        }

        /**
         * Checks if the table contains a value.
         * @param id The id of the value to check.
         * @return True if the value is contained, false otherwise.
         */
        static bool contains(const uint16_t id)
        {
            return idToPath.find(id) != idToPath.end();
        }

        /**
         * Checks if a value is of a certain type.
         * @param id The id of the value to check.
         * @return True if the value is of the specified type, false otherwise.
         */
        template <typename T>
        static bool isType(const uint16_t id)
        {
            if (!contains(id))
                return false;

            return idToPath.at(id).type() == typeid(T);
        }

        /**
         * Gets the value within the table.
         * @param id The id of the value to get.
         */
        template <typename T>
        static T get(const uint16_t id)
        {
            if (!contains(id))
                throw std::runtime_error("Value not found");
            return std::any_cast<T>(idToPath.at(id));
        }

    private:
        static std::map<uint16_t, std::any> idToPath;
    };
}

// Initialize the static member variable
std::map<uint16_t, std::any> vexbridge::table::ValueTable::idToPath = std::map<uint16_t, std::any>();