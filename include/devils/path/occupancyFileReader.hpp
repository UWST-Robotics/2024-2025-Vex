#pragma once
#include "occupancyGrid.hpp"
#include "../utils/logger.hpp"
#include "../utils/stringUtils.hpp"
#include "../geometry/units.hpp"
#include "../hardware/sdCard.hpp"
#include <fstream>
#include <iostream>

namespace devils
{
    /**
     * Reads an occupancy file from the SD card.
     */
    class OccupancyFileReader
    {
    public:
        /**
         * Deserializes an occupancy file from the SD card.
         * @return The deserialized occupancy file.
         */
        static OccupancyGrid readFromSD()
        {
            return deserialize(SDCard::readToString(OCCUPANCY_FILE_PATH));
        }

        /**
         * Deserializes a occupancy file from a string.
         * @param data The data to deserialize.
         * @return The deserialized occupancy file.
         */
        static OccupancyGrid deserialize(std::string data)
        {
            // Create a new occupancy grid
            OccupancyGrid grid;

            // Read from string
            std::string line;
            std::istringstream readStream(data);

            // Iterate through each line
            while (std::getline(readStream, line))
            {
                if (line.empty())
                    continue;
                if (line.rfind("ENDOCCUPANCY") == 0)
                    break;
                if (line.rfind("OCCUPANCY 1") == 0)
                    continue;

                // Update Dimensions
                grid.height = 0;
                grid.width++;

                for (char v : line)
                {
                    // Skip carriage returns
                    if (v == '\r' || v == '\n')
                        continue;
                    grid.height++;
                    grid.values.push_back(v == '1');
                }
            }

            return grid;
        }

    private:
        OccupancyFileReader() = delete;

        inline static const std::string OCCUPANCY_FILE_PATH = "occupancy.txt";
    };
}