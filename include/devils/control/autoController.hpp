#pragma once
#include "pros/rtos.hpp"
#include "../geometry/pose.hpp"
#include "../utils/runnable.hpp"
#include "../path/pathFile.hpp"

namespace devils
{
    /**
     * Represents a stateful system for controlling the robot chassis autonomously.
     */
    struct AutoController : public Runnable
    {
        /**
         * Represents the current state of the controller.
         */
        struct State
        {
            Pose *target = nullptr;
            PathEvents *events = &NO_EVENTS;
            bool isFinished = false;
            std::string debugText = "";
        };

        /**
         * Resets the controller.
         * Should be called right before the controller is run for timing purposes.
         */
        virtual void reset()
        {
            currentState.isFinished = false;
            currentState.target = nullptr;
            currentState.events = &NO_EVENTS;
        }

        /**
         * Updates the controller.
         * Should be called in a loop to run the controller. This is a direct alternative to `runSync` and `runAsync`.
         */
        virtual void update() override = 0;

        /**
         * Gets the current state of the controller.
         * Includes active events, targetPose, and any additional state information.
         * @return The current state of the controller as a `AutoController::State`.
         */
        virtual State &getState()
        {
            return currentState;
        }

        /**
         * Gets whether the controller is finished.
         * Alias for `getState().finished`.
         * @return True if the controller is finished, false otherwise.
         */
        bool getFinished()
        {
            return getState().isFinished;
        }

        /**
         * Runs the controller synchronously.
         * This will block the current thread until the controller is marked as finished.
         */
        void runSync() override
        {
            reset();
            while (!getFinished())
            {
                update();
                pros::delay(20);
            }
        }

        /**
         * Runs the controller as asyncronously
         * @return The PROS task that runs the controller.
         */
        pros::Task runAsync()
        {
            return pros::Task([=, this]
                              { runSync(); });
        }

    protected:
        static std::vector<PathEvent> NO_EVENTS;

        State currentState;
    };

    typedef AutoController::State AutoState;
}

// Define an empty vector of PathEvents
devils::PathEvents devils::AutoController::NO_EVENTS = {};