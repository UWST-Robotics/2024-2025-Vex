#pragma once
#include <string>
#include "motor.hpp"
#include "scuffPneumatic.hpp"

namespace devils
{
    /**
     * Represents a set of `ScuffPneumatic`s grouped together.
     */
    class ScuffPneumaticGroup
    {
    public:
        /**
         * Creates a new scuff pneumatic group.
         * @param name The name of the pneumatic group (for logging purposes)
         * @param ports The ADI ports of the pneumatics in the group
         */
        ScuffPneumaticGroup(std::string name, std::initializer_list<uint8_t> ports)
            : name(name), pneumatics()
        {
            pneumatics.reserve(ports.size());
            for (int8_t port : ports)
                pneumatics.push_back(std::make_shared<ScuffPneumatic>(_getPneumaticName(port), port));
        }

        /**
         * Extends all the pneumatics in the group.
         */
        void extend()
        {
            for (auto pneumatic : pneumatics)
                pneumatic->extend();
        }

        /**
         * Retracts all the pneumatics in the group.
         */
        void retract()
        {
            for (auto pneumatic : pneumatics)
                pneumatic->retract();
        }

        /**
         * Checks if all the pneumatics in the group are extended.
         * @return True if all the pneumatics in the group are extended, false otherwise.
         */
        bool getExtended()
        {
            for (auto pneumatic : pneumatics)
                if (!pneumatic->getExtended())
                    return false;
            return true;
        }

        /**
         * Gets the pneumatics in the motor group.
         * @return The pneumatics in the motor group.
         */
        std::vector<std::shared_ptr<ScuffPneumatic>> &getPneumatics()
        {
            return pneumatics;
        }

        /**
         * Gets the name of each pneumatic in the pneumatic group.
         */
        std::string _getPneumaticName(uint32_t port)
        {
            return name + "_" + std::to_string(port);
        }

    private:
        const std::string name;
        std::vector<std::shared_ptr<ScuffPneumatic>> pneumatics;
    };
}