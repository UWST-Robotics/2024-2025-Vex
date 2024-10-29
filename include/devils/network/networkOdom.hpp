#pragma once
#include "networkObject.hpp"
#include "../odom/odomSource.hpp"
#include "../network/networkTables.hpp"
#include "../geometry/units.hpp"

namespace devils
{
    class NetworkOdom : private INetworkObject
    {
    public:
        NetworkOdom(std::string name, OdomSource &odomSource)
            : name(name),
              odomSource(odomSource)
        {
        }

        void serialize() override
        {
            // Get Pose
            Pose &pose = odomSource.getPose();

            // Get Prefix
            std::string networkTableKey = "_poses/" + name;

            NetworkTables::UpdateValue(networkTableKey + "/name", name);
            NetworkTables::UpdateValue(networkTableKey + "/x", std::to_string(pose.x));
            NetworkTables::UpdateValue(networkTableKey + "/y", std::to_string(pose.y));
            NetworkTables::UpdateValue(networkTableKey + "/rotation", std::to_string(Units::radToDeg(pose.rotation)));
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
        std::string name;
        OdomSource &odomSource;
    };
}