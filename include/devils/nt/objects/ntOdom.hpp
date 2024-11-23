#pragma once

#include "../ntObjectBase.hpp"
#include "../networkTables.hpp"
#include "../../odom/odomSource.hpp"
#include "../../geometry/units.hpp"
#include "../../geometry/vector3.hpp"

namespace devils
{
    class NTOdom : private NTObjectBase
    {
    public:
        NTOdom(std::string name, OdomSource &odomSource)
            : name(name),
              odomSource(odomSource)
        {
        }

        void serialize() override
        {
            // Get Pose
            Pose &pose = odomSource.getPose();
            Vector3 velocity = calculateVelocity();

            // Get Prefix
            std::string networkTableKey = "_poses/" + name;

            NetworkTables::updateValue(networkTableKey + "/name", name);
            NetworkTables::updateDoubleValue(networkTableKey + "/x", pose.x);
            NetworkTables::updateDoubleValue(networkTableKey + "/y", pose.y);
            NetworkTables::updateDoubleValue(networkTableKey + "/rotation", Units::radToDeg(pose.rotation));
            NetworkTables::updateDoubleValue(networkTableKey + "/velocityX", velocity.x);
            NetworkTables::updateDoubleValue(networkTableKey + "/velocityY", velocity.y);
            NetworkTables::updateDoubleValue(networkTableKey + "/velocityRotation", velocity.z);
        }

        /**
         * Sets the size of the robot for rendering purposes.
         * @param width The width of the robot in inches.
         * @param length The length of the robot in inches.
         */
        void setSize(double width, double length)
        {
            NetworkTables::updateDoubleValue("_poses/" + name + "/width", width);
            NetworkTables::updateDoubleValue("_poses/" + name + "/length", length);
        }

    private:
        Pose lastPose;
        double lastTime;

        Vector3 calculateVelocity()
        {
            // Get Pose
            Pose &pose = odomSource.getPose();

            // Calculate Velocity
            double dt = pros::millis() - lastTime;
            double dx = pose.x - lastPose.x;
            double dy = pose.y - lastPose.y;
            double dr = pose.rotation - lastPose.rotation;

            // Update Last Pose
            lastPose = pose;
            lastTime = pros::millis();

            return Vector3(dx / dt, dy / dt, dr / dt);
        }

        std::string name;
        OdomSource &odomSource;
    };
}