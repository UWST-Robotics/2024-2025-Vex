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

            NetworkTables::UpdateValue(networkTableKey + "/name", name);
            NetworkTables::UpdateValue(networkTableKey + "/x", std::to_string(pose.x));
            NetworkTables::UpdateValue(networkTableKey + "/y", std::to_string(pose.y));
            NetworkTables::UpdateValue(networkTableKey + "/rotation", std::to_string(Units::radToDeg(pose.rotation)));
            NetworkTables::UpdateValue(networkTableKey + "/velocityX", std::to_string(velocity.x));
            NetworkTables::UpdateValue(networkTableKey + "/velocityY", std::to_string(velocity.y));
            NetworkTables::UpdateValue(networkTableKey + "/velocityRotation", std::to_string(velocity.z));
        }

        /**
         * Sets the size of the robot for rendering purposes.
         * @param width The width of the robot in inches.
         * @param length The length of the robot in inches.
         */
        void setSize(double width, double length)
        {
            NetworkTables::UpdateValue("_poses/" + name + "/width", std::to_string(width));
            NetworkTables::UpdateValue("_poses/" + name + "/length", std::to_string(length));
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