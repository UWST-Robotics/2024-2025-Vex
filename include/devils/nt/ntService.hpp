#pragma once

#include "../utils/runnable.hpp"
#include <vector>
#include "ntObjectBase.hpp"
#include "networkTables.hpp"

namespace devils
{
    /**
     * Singleton service for managing network objects.
     */
    class NTService : public AutoRunnable
    {
    public:
        /**
         * Gets the singleton instance of the network service.
         * @return The singleton instance of the network service.
         */
        static NTService &getInstance()
        {
            static NTService instance;
            return instance;
        }

        /**
         * Updates all network objects.
         */
        void update() override
        {
            // Serialize all network objects
            NetworkObjectList &allNetworkObjects = NTObjectBase::GetAllNetworkObjects();
            for (int i = 0; i < allNetworkObjects.size(); i++)
                if (allNetworkObjects[i]->isDirty())
                    allNetworkObjects[i]->runSerialization();
        }

    private:
        // Singleton Constructor
        NTService() : AutoRunnable() {}

    public:
        // Prevent copying
        NTService(NTService const &) = delete;
        void operator=(NTService const &) = delete;
    };
}