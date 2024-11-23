#pragma once

#include <string>
#include <sstream>
#include <vector>
#include "../geometry/pose.hpp"
#include "../utils/stringUtils.hpp"
#include "structs/splinePose.hpp"
#include "splinePath.hpp"

namespace devils
{
    class AutoDevilDeserializer
    {
    public:
        AutoDevilDeserializer() = delete;

        /**
         * Deserializes a path exported from AutoDevil
         * @param fileContents The contents of the file
         * @return The deserialized path
         */
        static SplinePath deserializePath(std::string fileContents)
        {
            // Path File
            std::vector<SplinePose> poses;

            // Input Stream
            std::istringstream stream(fileContents);

            // Iterate through lines
            for (std::string line; std::getline(stream, line);)
            {
                // Split line by spaces
                auto parts = StringUtils::split(line, ' ');
                if (parts.size() <= 0)
                    continue;

                // Path Header
                if (parts[0] == "PATH")
                {
                    if (parts.size() < 2)
                        continue;
                    if (parts[1] != "1")
                        throw std::runtime_error("Invalid AutoDevil path file version '" + parts[1] + "'");
                }

                // Point
                else if (parts[0] == "POINT")
                {
                    if (parts.size() < 6)
                        continue;
                    SplinePose pose;
                    pose.x = std::stod(parts[1]);
                    pose.y = std::stod(parts[2]);
                    pose.rotation = std::stod(parts[3]);
                    pose.entryDelta = std::stod(parts[4]);
                    pose.exitDelta = std::stod(parts[5]);
                    poses.push_back(pose);
                }

                // Reverse
                else if (parts[0] == "REVERSE")
                {
                    // TODO: Implement reverse
                }

                // Event
                else if (parts[0] == "EVENT")
                {
                    // TODO: Implement event
                }

                // Event
                else if (parts[0] == "ENDPATH")
                {
                    break;
                }

                // Unknown
                else
                {
                    throw std::runtime_error("Unknown AutoDevil path command '" + parts[0] + "'");
                }
            }

            return SplinePath(poses);
        }
    };
}