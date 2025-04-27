#pragma once

#include "odomSource.hpp"
#include "../utils/runnable.hpp"
#include "pros/rtos.hpp"
#include <queue>
#include <algorithm>

namespace devils
{
    /**
     * Delays the output pose of an odometry source by a specified amount of time.
     * Used for latency compensation in vision-based odometry.
     */
    class DelayedOdom : public OdomSource, public Runnable
    {
    public:
        /**
         * Constructs a new delayed odometry source.
         * @param baseOdom The base odometry source to use
         * @param delay The delay in milliseconds
         */
        DelayedOdom(
            OdomSource &baseOdom,
            double delay)
            : baseOdom(baseOdom),
              maxQueueSize(delay / INTERVAL_DELAY),
              Runnable(INTERVAL_DELAY)
        {
            // Ensure the queue size is at least 1
            if (maxQueueSize < 1)
                maxQueueSize = 1;

            // Check if the queue size is too large
            if (maxQueueSize > MAX_QUEUE_SIZE)
                maxQueueSize = MAX_QUEUE_SIZE;
        }

        void onUpdate() override
        {
            // Get the current pose from the base odometry source
            auto pose = baseOdom.getPose();
            auto velocity = baseOdom.getVelocity();

            // Push the pose to the queue
            poseQueue.push(pose);
            velocityQueue.push(velocity);

            // Check if the queue is too large
            while (poseQueue.size() > maxQueueSize)
                poseQueue.pop();

            while (velocityQueue.size() > maxQueueSize)
                velocityQueue.pop();
        }

        Pose getPose() override
        {
            // Check if the queue is empty
            if (poseQueue.empty())
                return baseOdom.getPose();

            // Get the pose from the front of the queue
            return poseQueue.front();
        }

        PoseVelocity getVelocity() override
        {
            // Check if the queue is empty
            if (velocityQueue.empty())
                return baseOdom.getVelocity();

            // Get the velocity from the front of the queue
            return velocityQueue.front();
        }

        void setPose(Pose pose) override
        {
            baseOdom.setPose(pose);
        }

    private:
        /// @brief Update interval in milliseconds
        static constexpr uint32_t INTERVAL_DELAY = 20;

        /// @brief Maximum number of poses to store in the queue
        static constexpr size_t MAX_QUEUE_SIZE = 100;

        std::queue<Pose> poseQueue;
        std::queue<PoseVelocity> velocityQueue;

        OdomSource &baseOdom;
        size_t maxQueueSize;
    };
}