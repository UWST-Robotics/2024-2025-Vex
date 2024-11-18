#pragma once
#include <string>
#include "motor.hpp"
#include "adiPneumatic.hpp"

namespace devils
{
    typedef std::vector<std::shared_ptr<ADIPneumatic>> ADIPneumaticList;

    /**
     * Represents a set of `ADIPneumatic`s grouped together.
     */
    class ADIPneumaticGroup
    {
    public:
        /**
         * Creates a new ADI pneumatic group.
         * @param name The name of the pneumatic group (for logging purposes)
         * @param ports The ADI ports of the pneumatics in the group
         */
        ADIPneumaticGroup(
            std::string name,
            std::initializer_list<uint8_t> ports)
            : name(name),
              pneumatics()
        {
            pneumatics.reserve(ports.size());
            for (int8_t port : ports)
                pneumatics.push_back(std::make_shared<ADIPneumatic>(getPneumaticName(port), port));
        }

        /**
         * Sets the state of all the pneumatics in the group.
         * @param isExtended True to extend the pneumatics, false to retract them.
         */
        void setExtended(bool isExtended)
        {
            for (auto pneumatic : pneumatics)
                pneumatic->setExtended(isExtended);
        }

        /**
         * Extends all the pneumatics in the group.
         */
        void extend()
        {
            setExtended(true);
        }

        /**
         * Retracts all the pneumatics in the group.
         */
        void retract()
        {
            setExtended(false);
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
        ADIPneumaticList &getPneumatics()
        {
            return pneumatics;
        }

    private:
        /**
         * Gets the name of each pneumatic in the pneumatic group.
         * @param port The port of the pneumatic
         * @return The name of the pneumatic
         */
        std::string getPneumaticName(uint32_t port)
        {
            return name + "_" + std::to_string(port);
        }

        const std::string name;
        ADIPneumaticList pneumatics;
    };
}