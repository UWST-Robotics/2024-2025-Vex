#pragma once
#include "pathFile.hpp"
#include "../utils/logger.hpp"
#include "../utils/stringUtils.hpp"
#include "../geometry/units.hpp"
#include "../hardware/sdCard.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

namespace devils
{
    /**
     * Reads a path file from the SD card.
     */
    class PathFileReader
    {
    public:
        /**
         * Deserializes a path file from the SD card.
         * @return The deserialized path file.
         */
        static PathFile readFromSD()
        {
            return deserialize(SDCard::readToString(PATH_FILE_PATH));
        }

        /**
         * Deserializes a path file from a string.
         * @param data The data to deserialize.
         * @return The deserialized path file.
         */
        static PathFile deserialize(std::string data)
        {
            // Create a new path file
            PathFile pathFile;
            pathFile.version = 1;

            // Read from string
            std::string line;
            std::istringstream readStream(data);

            // Handle reverse
            bool isReversed = false;

            // Iterate through each line
            while (std::getline(readStream, line))
            {
                if (line.empty())
                    continue;
                if (line.rfind("ENDPATH") == 0)
                    break;
                if (line.rfind("PATH 1") == 0)
                    continue;
                if (line.rfind("POINT") == 0)
                {
                    ControlPoint point = _parsePoint(line);
                    point.isReversed = isReversed;
                    pathFile.points.push_back(point);
                }
                if (line.rfind("EVENT") == 0)
                {
                    PathEvent event = _parseEvent(line);
                    pathFile.points.back().events.push_back(event);
                }
                if (line.rfind("REVERSE") == 0)
                {
                    isReversed = !isReversed;
                    pathFile.points.back().isReversed = isReversed;
                }
            }
            return pathFile;
        }

        /**
         * Parses a point from a line in the path file.
         * @param line The line to parse.
         * @return The parsed point.
         */
        static ControlPoint _parsePoint(std::string line)
        {
            // Split the line into properties
            auto split = StringUtils::split(line, ' ');
            int index = 0;
            ControlPoint point;

            // Iterate through each property
            for (int i = 0; i < split.size(); i++)
            {
                // Ignore POINT
                if (split[i].rfind("POINT") == 0)
                    continue;
                // Parse Index to Property
                if (index == 0)
                    point.x = std::stof(split[i]);
                if (index == 1)
                    point.y = std::stof(split[i]);
                if (index == 2)
                    point.rotation = std::stof(split[i]);
                if (index == 3)
                    point.enterDelta = std::stof(split[i]);
                if (index == 4)
                    point.exitDelta = std::stof(split[i]);
                // Increment Index
                index++;
            }

            // Return the point
            return point;
        }

        /**
         * Parses an event from a line in the path file.
         * @param line The line to parse.
         * @return The parsed event.
         */
        static PathEvent _parseEvent(std::string line)
        {
            // Split the line into properties
            auto split = StringUtils::split(line, ' ');
            int index = 0;
            PathEvent event = PathEvent("", "");

            // Iterate through each property
            for (int i = 0; i < split.size(); i++)
            {
                // Ignore EVENT
                if (split[i].rfind("EVENT") == 0)
                    continue;
                // Parse Index to Property
                if (index == 0)
                    event.name = split[i];
                if (index == 1)
                    event.params = split[i];
                // Increment Index
                index++;
            }

            // Return the event
            return event;
        }

    private:
        PathFileReader() = delete;

        inline static const std::string PATH_FILE_PATH = "path.txt";
    };
}