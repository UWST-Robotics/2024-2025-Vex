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
    class NTService : public Runnable
    {
    public:
        // Constructor
        NTService()
        {
            // Call Reset
            NetworkTables::reset();

            // Run on startup
            runAsync();
        }

        /**
         * Updates all network objects.
         */
        void onUpdate() override
        {
            // Heartbeat
            NetworkTables::sendHeartbeat();

            // Serialize all network objects
            auto allNetworkObjects = NTObjectBase::getAllNetworkObjects();
            for (auto obj : *allNetworkObjects)
                if (obj->isDirty())
                    obj->runSerialization();
        }
    };
}