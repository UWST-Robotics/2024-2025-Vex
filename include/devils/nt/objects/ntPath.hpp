#pragma once

#include "../ntObjectBase.hpp"
#include "../networkTables.hpp"
#include "../../path/path.hpp"

namespace devils
{
    class NTPath : private NTObjectBase
    {
    public:
        NTPath(std::string name, Path &path)
            : name(name),
              path(path)
        {
            setSerializationRate(SERIALIZATION_RATE);
        }

        void serialize() override
        {
            // Get Prefix
            std::string networkTableKey = "_lines/" + name + "/";

            // Iterate over the path
            for (double i = 0; i < path.getLength() - 1; i += DELTA_T)
            {
                // Get Pose
                Pose pose = path.getPoseAt(i);
                int index = i / DELTA_T;

                // Update Network Table
                NetworkTables::updateDoubleValue(networkTableKey + std::to_string(index) + "/x", pose.x);
                NetworkTables::updateDoubleValue(networkTableKey + std::to_string(index) + "/y", pose.y);
            }
        }

    private:
        static constexpr double DELTA_T = 0.1;             // % of segment length
        static constexpr double SERIALIZATION_RATE = 5000; // ms

        std::string name;
        Path &path;
    };
}