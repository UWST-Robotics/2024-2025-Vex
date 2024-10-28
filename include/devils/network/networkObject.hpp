#pragma once
#include "pros/rtos.hpp"

namespace devils
{
    class INetworkObject;
    typedef std::vector<devils::INetworkObject *> NetworkObjectList;

    /**
     * Represents a netowrk object that can be serialized and sent over serial.
     */
    struct INetworkObject
    {

        // Register the network object
        INetworkObject()
        {
            GetAllNetworkObjects().push_back(this);
        }

        // Unregister the network object
        ~INetworkObject()
        {
            NetworkObjectList &allNetworkObjects = GetAllNetworkObjects();

            allNetworkObjects.erase(
                std::remove(allNetworkObjects.begin(), allNetworkObjects.end(), this),
                allNetworkObjects.end());
        }

        // Disable copy constructor
        INetworkObject(INetworkObject const &) = delete;

        /**
         * Serializes the object to send over serial.
         * Should call the `NetworkTable::UpdateValue` method to send the data.
         */
        virtual void serialize() = 0;

        /**
         * Gets a list of all alive network objects.
         * @return A list of all alive network objects.
         */
        static NetworkObjectList &GetAllNetworkObjects()
        {
            static NetworkObjectList allNetworkObjects;
            return allNetworkObjects;
        }
    };
}