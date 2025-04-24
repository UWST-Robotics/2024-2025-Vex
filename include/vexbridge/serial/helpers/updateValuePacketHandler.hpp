#pragma once

#include <cstdint>
#include <deque>
#include <memory>
#include "../../table/valueTable.hpp"
#include "../packetTypes/updateValuePacket.hpp"
#include "../packetTypes/updateBoolPacket.hpp"
#include "../packetTypes/updateIntPacket.hpp"
#include "../packetTypes/updateFloatPacket.hpp"
#include "../packetTypes/updateDoublePacket.hpp"
#include "../packetTypes/updateStringPacket.hpp"
#include "../packetTypes/updateBoolArrayPacket.hpp"
#include "../packetTypes/updateIntArrayPacket.hpp"
#include "../packetTypes/updateFloatArrayPacket.hpp"
#include "../packetTypes/updateDoubleArrayPacket.hpp"

using namespace vexbridge::table;

namespace vexbridge::serial
{
    struct UpdateValuePacketHandler
    {
        /**
         * Checks if a packet is an `UpdateValuePacket`.
         * If it is, it will update its value in the value table.
         * @param newPacket The new packet to update.
         */
        static void handlePacket(SerialPacket *newPacket)
        {
            tryUpdateValue<UpdateBoolPacket, bool>(newPacket);
            tryUpdateValue<UpdateIntPacket, int>(newPacket);
            tryUpdateValue<UpdateFloatPacket, float>(newPacket);
            tryUpdateValue<UpdateDoublePacket, double>(newPacket);
            tryUpdateValue<UpdateStringPacket, std::string>(newPacket);
            tryUpdateValue<UpdateBoolArrayPacket, std::vector<bool>>(newPacket);
            tryUpdateValue<UpdateIntArrayPacket, std::vector<int>>(newPacket);
            tryUpdateValue<UpdateFloatArrayPacket, std::vector<float>>(newPacket);
            tryUpdateValue<UpdateDoubleArrayPacket, std::vector<double>>(newPacket);
        }

    private:
        /**
         * Tries to update the value of a packet depending on its type.
         * This is necessary since C++ does not support dynamic casting of templated types.
         * @param serialPacket The packet to update the value of.
         */
        template <typename T, typename U>
        static void tryUpdateValue(SerialPacket *serialPacket)
        {
            if (auto updatePacket = dynamic_cast<UpdateValuePacket<U> *>(serialPacket))
                ValueTable::set(updatePacket->valueID, updatePacket->newValue);
        }
    };
}