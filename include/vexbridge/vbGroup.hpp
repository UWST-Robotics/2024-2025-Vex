#pragma once

#include <string>
#include "vbValue.hpp"

namespace vexbridge
{
    /**
     * Factory for creating VBValues.
     */
    class VBGroup
    {
    public:
        /**
         * Creates a new group with a path.
         * @param path The path of the group.
         */
        VBGroup(const std::string path)
            : path(path)
        {
        }

        /**
         * Creates a new value as a child of this group.
         * @param label The label of the value.
         * @param defaultValue The default value.
         */
        template <typename T>
        VBValue<T> addValue(const std::string label, const T defaultValue) const
        {
            return VBValue<T>(path + "/" + label, defaultValue);
        }

        /**
         * Creates a new VBGroup as a child of this group.
         * @param label The label of the child group.
         */
        VBGroup addGroup(const std::string label) const
        {
            return VBGroup(path + "/" + label);
        }

    private:
        const std::string path;
    };
}