#pragma once

#include "devils/devils.h"

namespace devils
{
    /**
     * Represents the goal rush system on the back of the robot.
     */
    class GoalRushSystem
    {
    public:
        GoalRushSystem(
            ADIPneumatic &deployPneumatic,
            ADIPneumatic &clampPneumatic,
            ADIDigitalInput &goalRushSensor)
            : deployPneumatic(deployPneumatic),
              clampPneumatic(clampPneumatic),
              goalRushSensor(goalRushSensor)
        {
        }

        /**
         * Checks if there is a mogo present in the goal rush.
         * @return True if a mogo is present, false otherwise.
         */
        bool hasMogo()
        {
            return goalRushSensor.getValue();
        }

        /**
         * Gets whether the goal rush is extended.
         * @return True if the goal rush is extended, false otherwise.
         */
        bool getExtended()
        {
            return deployPneumatic.getExtended();
        }

        /**
         * Gets whether the goal rush is clamped.
         * @return True if the goal rush is clamped, false otherwise.
         */
        bool getClamped()
        {
            return clampPneumatic.getExtended();
        }

        /**
         * Sets whether the goal rush is extended.
         * @param isExtended True to extend the goal rush, false to retract it.
         */
        void setExtended(bool isExtended)
        {
            deployPneumatic.setExtended(isExtended);
        }

        /**
         * Sets whether the goal rush is clamped.
         * @param isClamped True to clamp the goal rush, false to un-clamp it.
         */
        void setClamped(bool isClamped)
        {
            clampPneumatic.setExtended(isClamped);
        }

    private:
        ADIPneumatic &clampPneumatic;
        ADIPneumatic &deployPneumatic;
        ADIDigitalInput &goalRushSensor;
    };
}