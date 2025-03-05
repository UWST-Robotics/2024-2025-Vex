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
        GoalRushSystem(ADIPneumatic &goalRushPneumatic)
            : goalRushPneumatic(goalRushPneumatic)
        {
        }

        /**
         * Gets whether the goal rush is extended.
         * @return True if the goal rush is extended, false otherwise.
         */
        bool isGoalRushExtended()
        {
            return goalRushPneumatic.getExtended();
        }

        void setGoalRushExtended(bool isExtended)
        {
            goalRushPneumatic.setExtended(isExtended);
        }

        private:
          ADIPneumatic &goalRushPneumatic;
          ADIDigitalInput *goalRushSensor = nullptr;
    };
}