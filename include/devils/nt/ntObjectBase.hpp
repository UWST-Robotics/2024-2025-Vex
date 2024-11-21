#pragma once

#include <unordered_set>
#include "pros/rtos.hpp"

namespace devils
{
    class NTObjectBase;
    typedef std::unordered_set<devils::NTObjectBase *> NetworkObjectList;

    /**
     * Represents a netowrk object that can be serialized and sent over serial.
     */
    class NTObjectBase
    {
    public:
        // Register the network object
        NTObjectBase()
        {
            mutex.take(0);
            allNetworkObjects.insert(this);
            mutex.give();
        }

        // Copy constructor
        NTObjectBase(NTObjectBase const &)
        {
            mutex.take(0);
            allNetworkObjects.insert(this);
            mutex.give();
        }

        // Unregister the network object
        ~NTObjectBase()
        {
            mutex.take(0);
            allNetworkObjects.erase(this);
            mutex.give();
        }

        /**
         * Sets the rate at which the object should be serialized.
         * Increasing the rate will decrease the frequency of serialization for lower priority objects.
         * @param maxSerializeTime The rate at which the object should be serialized in milliseconds.
         */
        void setSerializationRate(int maxSerializeTime)
        {
            this->maxSerializeTime = maxSerializeTime;
        }

        /**
         * Checks if the object is dirty
         * @return True if the object is dirty, false otherwise.
         */
        bool isDirty()
        {
            if (lastSerializationTime == -1)
                return true;
            return pros::millis() - lastSerializationTime > maxSerializeTime;
        }

        /**
         * Runs the serialization of the object.
         * Updates the last serialization time to clean the object.
         * Usually called automatically in `NetworkService::update`.
         */
        void runSerialization()
        {
            serialize();
            lastSerializationTime = pros::millis();
        }

        static NetworkObjectList *getAllNetworkObjects()
        {
            return &allNetworkObjects;
        }

    protected:
        /**
         * Should be implemented by child classes to serialize the object.
         * Child classes can call the `NetworkTable::UpdateValue` method to update the data.
         */
        virtual void serialize() {};

    private:
        static NetworkObjectList allNetworkObjects;
        static pros::Mutex mutex;

        int maxSerializeTime = 50; // ms
        uint32_t lastSerializationTime = -1;
    };
}

// Initialize static variables
devils::NetworkObjectList devils::NTObjectBase::allNetworkObjects;
pros::Mutex devils::NTObjectBase::mutex;