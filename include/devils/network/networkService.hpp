#pragma once
#include "../utils/runnable.hpp"
#include <vector>
#include "networkObject.hpp"
#include "networkTables.hpp"

namespace devils
{
    /**
     * Singleton service for managing network objects.
     */
    class NetworkService : public AutoRunnable
    {
    public:
        /**
         * Gets the singleton instance of the network service.
         * @return The singleton instance of the network service.
         */
        static NetworkService &getInstance()
        {
            static NetworkService instance;
            return instance;
        }

        /**
         * Updates all network objects.
         */
        void update() override
        {
            // Serialize all network objects
            NetworkObjectList &allNetworkObjects = INetworkObject::GetAllNetworkObjects();
            for (int i = 0; i < allNetworkObjects.size(); i++)
                if (allNetworkObjects[i]->isDirty())
                    allNetworkObjects[i]->runSerialization();
        }

    private:
        // Singleton Constructor
        NetworkService() : AutoRunnable() {}

    public:
        // Prevent copying
        NetworkService(NetworkService const &) = delete;
        void operator=(NetworkService const &) = delete;
    };
}