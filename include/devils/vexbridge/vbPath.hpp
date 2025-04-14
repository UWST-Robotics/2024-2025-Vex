#pragma once
#include "../path/path.hpp"
#include "vexbridge/vbValue.hpp"
#include "vexbridge/vbGroup.hpp"

using namespace vexbridge;

namespace devils
{
    /**
     * Syncs path data to VEXBridge.
     */
    class VBPath
    {
    public:
        /**
         * Updates the path data on VEXBridge.
         * @param name The name of the path.
         * @param path The path to sync.
         */
        static void sync(std::string name, Path &path)
        {
            // Create vectors for x and y values
            std::vector<float> xValues;
            std::vector<float> yValues;

            xValues.reserve(path.getLength() / DELTA_INDEX);
            yValues.reserve(path.getLength() / DELTA_INDEX);

            // Iterate over the path and add points
            for (double i = 0; i < path.getLength(); i += DELTA_INDEX)
            {
                Pose pose = path.getPoseAt(i);

                xValues.push_back(pose.x);
                yValues.push_back(pose.y);
            }

            // Sync the path data
            VEXBridge::set("_paths/" + name + "/x", xValues);
            VEXBridge::set("_paths/" + name + "/y", yValues);
        }

    private:
        static constexpr double DELTA_INDEX = 0.1;
    };
}