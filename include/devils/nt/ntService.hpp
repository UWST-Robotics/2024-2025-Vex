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
        NTService() : Runnable(100)
        {
            // Call Reset
            NetworkTables::reset();

            // Run on startup
            if (ENABLE_SERVICE)
                runAsync();
        }

        /**
         * Updates all network objects.
         */
        void onUpdate() override
        {
            // Heartbeat
            // NetworkTables::sendHeartbeat();

            // Serialize all network objects
            auto allNetworkObjects = NTObjectBase::getAllNetworkObjects();
            for (auto obj : *allNetworkObjects)
                if (obj->isDirty())
                    obj->runSerialization();
        }

    private:
        // Disable during competition to prevent serial spam
        // This causes intermittent crashing on the V5 brain
        static constexpr bool ENABLE_SERVICE = false;
    };
}