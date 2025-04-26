#pragma once
#include "devils/devils.h"
#include <queue>

namespace devils
{
    /**
     * Controls the LED lights on the robot.
     */
    class HornLEDSystem : public Daemon
    {
    public:
        /**
         * Constructs a new HornLEDSystem with the given ports.
         * @param leftLEDPort The ADI port of the left LED (from 1 to 8)
         * @param rightLEDPort The ADI port of the right LED (from 1 to 8)
         */
        HornLEDSystem(LED &leftLED, LED &rightLED)
            : leftLED(leftLED),
              rightLED(rightLED)
        {
        }

        /**
         * Resets the queue
         */
        void resetQueue()
        {
            queueMutex.take();

            // Clear the queues
            leftQueue = std::queue<bool>();
            rightQueue = std::queue<bool>();

            queueMutex.give();
        }

        /**
         * Pushes the state of the LEDs to the queue.
         * @param leftState The state of the left LED (true = on, false = off)
         * @param rightState The state of the right LED (true = on, false = off)
         */
        void pushState(bool leftState, bool rightState)
        {
            queueMutex.take();

            // Push the states to the queues
            leftQueue.push(leftState);
            rightQueue.push(rightState);

            queueMutex.give();
        }

        /**
         * Enables or disables the loop feature.
         * @param enabled True to enable the loop feature, false to disable it.
         */
        void setLoopEnabled(bool enabled)
        {
            queueMutex.take();
            loopEnabled = enabled;
            queueMutex.give();
        }

    protected:
        void update() override
        {
            queueMutex.take(); // <-- Avoid race conditions

            // Check if the left LED should be on
            if (!leftQueue.empty())
            {
                // Loop Queue
                if (loopEnabled)
                    leftQueue.push(leftQueue.front());

                // Update LED state
                leftLED.setEnabled(leftQueue.front());

                // Pop the front of the queue
                leftQueue.pop();
            }

            // Check if the right LED should be on
            if (!rightQueue.empty())
            {
                // Loop Queue
                if (loopEnabled)
                    rightQueue.push(rightQueue.front());

                // Update LED state
                rightLED.setEnabled(rightQueue.front());

                // Pop the front of the queue
                rightQueue.pop();
            }

            queueMutex.give(); // <-- Avoid race conditions

            // Wait for the next interval
            pros::delay(INTERVAL_DELAY);
        }

    private:
        static constexpr int MAX_QUEUE_SIZE = 10;  // Maximum size of the queue
        static constexpr int INTERVAL_DELAY = 250; // Delay between LED state changes in milliseconds

        pros::Mutex queueMutex;

        /// @brief When enabled, queue will loop continuously
        bool loopEnabled = true;

        std::queue<bool> leftQueue;
        std::queue<bool> rightQueue;

        LED &leftLED;
        LED &rightLED;
    };
}