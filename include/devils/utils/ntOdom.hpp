#pragma once
#include "../odom/odomSource.hpp"
#include "../geometry/units.hpp"
#include "runnable.hpp"
#include "vexbridge/vexbridge.h"

namespace devils
{
    /**
     * Syncs odometry data to VEXBridge.
     */
    class NTOdom : private Runnable
    {
    public:
        /**
         * Creates a new odometry syncer.
         * @param name The name of the odometry data.
         * @param odometry The odometry source to sync.
         */
        NTOdom(std::string name, OdomSource &odometry)
            : odometry(odometry),
              group("_poses/" + name),
              xValue(group.makeValue("x", 0.0f)),
              yValue(group.makeValue("y", 0.0f)),
              rotation(group.makeValue("rotation", 0.0f)),
              width(group.makeValue("width", 15.0f)),
              length(group.makeValue("length", 15.0f)),
              speed(group.makeValue("speed", 0.0f))
        {
            this->runAsync();
        }

        /**
         * Sets the dimensions of the robot.
         * @param width The width of the robot in inches.
         * @param length The length of the robot in inches.
         */
        void setDimensions(float width, float length)
        {
            this->width.set(width);
            this->length.set(length);
        }

    protected:
        void onUpdate() override
        {
            Pose pose = odometry.getPose();
            xValue.set(pose.x);
            yValue.set(pose.y);
            rotation.set(Units::radToDeg(pose.rotation));
            speed.set(odometry.getVelocity().magnitude());
        }

    private:
        NTGroup group;
        NTValue<float> xValue;
        NTValue<float> yValue;
        NTValue<float> rotation;
        NTValue<float> width;
        NTValue<float> length;
        NTValue<float> speed;
        OdomSource &odometry;
    };
}