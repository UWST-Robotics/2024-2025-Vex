#pragma once

#include <map>
#include <cstdint>
#include <any>

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
        void set(const uint16_t id, const T value)
        {
            idToPath[id] = value;
        }

        /**
         * Checks if the table contains a value.
         * @param id The id of the value to check.
         * @return True if the value is contained, false otherwise.
         */
        bool contains(const uint16_t id) const
        {
            return idToPath.find(id) != idToPath.end();
        }

        /**
         * Checks if a value is of a certain type.
         * @param id The id of the value to check.
         * @return True if the value is of the specified type, false otherwise.
         */
        template <typename T>
        bool isType(const uint16_t id) const
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
        T get(const uint16_t id) const
        {
            if (!contains(id))
                throw std::runtime_error("Value not found");
            return std::any_cast<T>(idToPath.at(id));
        }

    private:
        std::map<uint16_t, std::any> idToPath;
    };
}