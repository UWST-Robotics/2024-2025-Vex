#pragma once

#include "devils/devils.h"

namespace devils
{
    /**
     * Represents the mogo clamp on the back of the robot.
     */
    class MogoGrabSystem
    {
    public:
        MogoGrabSystem(ADIPneumatic &mogoPneumatic)
            : mogoPneumatic(mogoPneumatic)
        {
        }

        /**
         * Gets whether the mogo is grabbed.
         * @return True if the mogo is grabbed, false otherwise.
         */
        bool getMogoGrabbed()
        {
            return mogoPneumatic.getExtended();
        }

        /**
         * Checks if the mogo is in the robot. Defaults to true if no limit switch is connected.
         * @return True if the mogo is in the robot, false otherwise.
         */
        bool hasMogo()
        {
            if (mogoSensor == nullptr)
                return true;
            return mogoSensor->getValue();
        }

        /**
         * Sets whether the mogo is grabbed.
         * @param isGrabbed True if the mogo is grabbed, false otherwise.
         */
        void setMogoGrabbed(bool isGrabbed)
        {
            mogoPneumatic.setExtended(isGrabbed);
        }

        /**
         * Uses an ir/limit switch to determine if the mogo is in the robot.
         * @param sensor The limit switch to use.
         */
        void useSensor(ADIDigitalInput *sensor)
        {
            mogoSensor = sensor;
        }

    private:
        ADIPneumatic &mogoPneumatic;
        ADIDigitalInput *mogoSensor = nullptr;
    };
}