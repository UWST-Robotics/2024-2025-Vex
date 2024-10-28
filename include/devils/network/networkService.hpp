#pragma once
#include "../utils/runnable.hpp"
#include <vector>
#include "networkObject.hpp"

namespace devils
{
    /**
     * Singleton service for managing network objects.
     */
    class NetworkService : public Runnable
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
            NetworkObjectList &allNetworkObjects = INetworkObject::GetAllNetworkObjects();
            for (int i = 0; i < allNetworkObjects.size(); i++)
                allNetworkObjects[i]->serialize();
        }

    private:
        // Singleton Constructor
        NetworkService() : Runnable(UPDATE_INTERVAL)
        {
            if (NETWORK_SERVICE_ENABLED)
                runAsync();
        }

        // Constants
        static constexpr int UPDATE_INTERVAL = 200; // ms
        static constexpr bool NETWORK_SERVICE_ENABLED = true;

    public:
        // Prevent copying
        NetworkService(NetworkService const &) = delete;
        void operator=(NetworkService const &) = delete;
    };
}