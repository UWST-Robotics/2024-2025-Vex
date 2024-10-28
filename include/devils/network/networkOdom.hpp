#pragma once
#include "networkObject.hpp"
#include "../odom/odomSource.hpp"
#include "../network/networkTables.hpp"

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
            NetworkTables::UpdateValue(networkTableKey + "/rotation", std::to_string(pose.rotation));
        }

    private:
        std::string name;
        OdomSource &odomSource;
    };
}