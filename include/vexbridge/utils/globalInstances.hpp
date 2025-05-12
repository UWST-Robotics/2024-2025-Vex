#pragma once

#include <unordered_set>

namespace vexbridge::utils
{
    /**
     * Maintains a list of all instances of a class.
     */
    template <typename T>
    struct GlobalInstances
    {
        // Constructor
        GlobalInstances()
        {
            // Add this instance to the set of all instances
            allInstances.insert(static_cast<T *>(this));
        }

        // Copy Constructor (Calls the default constructor)
        GlobalInstances(const GlobalInstances &other) : GlobalInstances()
        {
        }

        // Move Constructor (Calls the default constructor)
        GlobalInstances(GlobalInstances &&other) : GlobalInstances()
        {
        }

        // Destructor
        ~GlobalInstances()
        {
            // Remove this instance from the set of all instances
            allInstances.erase(static_cast<T *>(this));
        }

        /// @brief Contains a list of all instances of the class.
        static inline std::unordered_set<T *> allInstances;
    };
}