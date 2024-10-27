#pragma once
#include "pros/misc.hpp"
#include "../utils/logger.hpp"

namespace devils
{
    struct SDCard
    {
        /**
         * Checks if the SD card is inserted into the brain.
         * @returns True if the SD card is insered, false otherwise.
         */
        static bool isInserted()
        {
            return pros::usd::is_installed();
        }

        /**
         * Reads a raw file from the SD card and returns it as a string. Each line is seperated by \n
         * @param fileName The name of the file to read.
         * @return The raw file as a string.
         */
        static std::string readToString(std::string fileName)
        {
            if (!isInserted())
            {
                Logger::error("SDCard::readFromSD: SD card is not installed!");
                return "";
            }

            // Open Stream
            std::ifstream file("/usd/" + fileName);
            std::string line;
            std::string data = "";

            // Iterate through each line
            // TODO: While this works for small files, this is a very memory-intensive algorithm
            while (std::getline(file, line))
                data += line + "\n";
            file.close();

            // Return Data
            return data;
        }
    };
}