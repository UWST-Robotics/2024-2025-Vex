#pragma once

#include <string>
#include "ntValue.hpp"

namespace bluebox
{
    class NTGroup
    {
    public:
        NTGroup(std::string path)
            : path(path)
        {
        }

        /**
         * Creates a new value in the group.
         * @param name The name of the value.
         * @param defaultValue The default value of the value.
         */
        template <typename T>
        NTValue<T> makeValue(std::string name, T defaultValue)
        {
            return NTValue<T>(path + "/" + name, defaultValue);
        }

        /**
         * Creates a new child group.
         * @param name The name of the child group.
         */
        NTGroup makeChild(std::string name)
        {
            return NTGroup(path + "/" + name);
        }

    private:
        std::string path;
    };
}