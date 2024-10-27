#pragma once
#include "pros/rtos.hpp"
#include "../path/generatedPath.hpp"
#include "../gameobject/gameObjectManager.hpp"
#include "../chassis/chassis.hpp"
#include "../odom/odomSource.hpp"
#include "../hardware/opticalSensor.hpp"
#include "../geometry/polygon.hpp"
#include "../utils/pid.hpp"
#include "autoController.hpp"

namespace devils
{
    /**
     * Runs though a list of controllers in sequential order
     */
    class ControllerList : public AutoController
    {
    public:
        /**
         * Constructs a new ControllerList.
         * @param controllers The controllers to run.
         * @param loop Whether to loop the controllers.
         * @param timeout Maximum time to run the controllers before marking as finished. Measured in milliseconds.
         */
        ControllerList(std::initializer_list<AutoController *> controllers, bool loop = false, int timeout = -1)
            : controllers(controllers),
              loop(loop),
              timeout(timeout)
        {
        }

        void reset() override
        {
            AutoController::reset();
            startTime = pros::millis();
            controllerIndex = 0;
            for (AutoController *controller : controllers)
                controller->reset();
        }

        void update() override
        {
            // Start the timeout timer
            if (startTime < 0)
                startTime = pros::millis();

            // Check if the controller is finished
            if (getFinished())
                return;

            // Update the current controller
            AutoController *controller = getCurrentController();
            if (controller != nullptr)
            {
                controller->update();
                if (controller->getFinished())
                    skip();
            }
        }

        void runSync() override
        {
            reset();
            while (!getFinished())
            {
                getCurrentController()->reset();
                getCurrentController()->runSync();
                skip();
                pros::delay(20);
            }
        }

        AutoController::State &getState() override
        {
            // Update State
            AutoController *controller = getCurrentController();
            if (controller != nullptr)
            {
                AutoState &currentControllerState = controller->getState();
                currentState.target = currentControllerState.target;
                currentState.events = currentControllerState.events;
                currentState.debugText = currentControllerState.debugText;
            }
            else
            {
                currentState.target = nullptr;
                currentState.events = &NO_EVENTS;
            }

            // Check for Timeout
            if (timeout > 0 && pros::millis() - startTime > timeout)
                currentState.isFinished = true;

            return currentState;
        }

        /**
         * Gets the current working controller.
         * @param searchChildren Recursively runs `getCurrentController` on each `ControllerList`.
         * @return The current controller
         */
        AutoController *getCurrentController(bool searchChildren = false)
        {
            AutoController *currentController = controllers[controllerIndex % controllers.size()];

            // Abort if not searching children
            if (!searchChildren)
                return currentController;

            // Loop through controller lists
            // Exits if it ever finds this controller
            ControllerList *list = nullptr;
            do
            {
                list = dynamic_cast<ControllerList *>(currentController);
                if (list != nullptr)
                    currentController = list->getCurrentController();
            } while (list != nullptr);

            return currentController;
        }

        /**
         * Skips the current controller and moves to the next one.
         */
        void skip()
        {
            // Increment the controller index
            controllerIndex++;

            // Update State
            if (!loop)
                currentState.isFinished = controllerIndex >= controllers.size();

            // Reset the next controller
            AutoController *currentController = getCurrentController();
            if (currentController != nullptr)
                currentController->reset();
        }

    private:
        std::vector<AutoController *> controllers;
        bool loop = false;
        int controllerIndex = 0;
        int timeout = -1;
        int startTime = -1;
    };
}