#pragma once
#include "pros/misc.hpp"
#include "pros/llemu.hpp"
#include <string>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <sstream>

namespace devils
{
    // TODO: Check to see if this logs to the SD card or if it only logs to the terminal

    /**
     * Represents a global logging utility.
     */
    class Logger
    {
    public:
        /**
         * Checks if the SD card is inserted.
         * @return True if the SD card is inserted.
         */
        static bool isSDInserted()
        {
            return pros::usd::is_installed() == 1;
        }

        /**
         * Initializes the logger.
         */
        static void init()
        {
            pros::lcd::initialize();
            // TODO: Fix logging
            // okapi::Logger::setDefaultLogger(std::make_shared<okapi::Logger>(
            //     okapi::TimeUtilFactory::createDefault().getTimer(),
            //     LOG_TERMINAL,
            //     okapi::Logger::LogLevel::debug));

            // Open Log File
            if (isSDInserted() && LOG_TO_SD)
            {
                std::string path = _getLogFilePath();
                logFileStream.open(path);
                if (logFileStream.is_open())
                    info("Logger: Opened log file at " + path);
                else
                    warn("Logger: Failed to open log file at " + path);
            }
            else if (LOG_TO_SD)
            {
                warn("Logger: SD card is not installed!");
            }
        }

        /**
         * Gets a file path for a new log file. Increments the index until a file that doesn't exist is found.
         * @return The file path for a new log file.
         */
        static std::string _getLogFilePath()
        {
            int index = 0;
            std::string path;
            do
            {
                path = "/usd/log-" + std::to_string(index) + ".txt";
                index++;
                info("Logger: Checking for log file at " + path);
            } while (std::ifstream(path).good());
            return path;
        }

        /**
         * Logs a message to the SD card.
         * @param message The message to log.
         */
        static void _logToSD(std::string message)
        {
            if (logFileStream.is_open())
            {
                logFileStream << message << std::endl;
                logFileStream.flush();
            }
        }

        /**
         * Logs a message to the LCD.
         * @param message The message to send.
         */
        static void _logToLCD(std::string message)
        {
            static int line = 0;
            pros::lcd::set_text(line, message);
            line = (line + 1) % 8;
        }

        /**
         * Logs an info message.
         * @param message The message to log.
         */
        static void info(std::string message)
        {
            // okapi::Logger::getDefaultLogger()->info([=]()
            //                                         { return std::string(message); });
            if (LOG_TO_DISPLAY)
                _logToLCD(message);
            if (LOG_TO_SD)
                _logToSD(message);
        }

        /**
         * Logs a warning message.
         * @param message The message to log.
         */
        static void warn(std::string message)
        {
            // okapi::Logger::getDefaultLogger()->warn([=]()
            //                                         { return std::string(message); });
            if (LOG_TO_DISPLAY)
                _logToLCD(message);
            if (LOG_TO_SD)
                _logToSD(message);
        }

        /**
         * Logs an error message.
         * @param message The message to log.
         */
        static void error(std::string message)
        {
            // okapi::Logger::getDefaultLogger()->error([=]()
            //                                          { return std::string(message); });
            if (LOG_TO_DISPLAY)
                _logToLCD(message);
            if (LOG_TO_SD)
                _logToSD(message);
        }

        /**
         * Logs a debug message.
         * @param message The message to log.
         */
        static void debug(std::string message)
        {
            // okapi::Logger::getDefaultLogger()->debug([=]()
            //                                          { return std::string(message); });
            if (LOG_TO_DISPLAY)
                _logToLCD(message);
            if (LOG_TO_SD)
                _logToSD(message);
        }

        // /**
        //  * Gets the Okapi logger.
        //  * @return The Okapi logger.
        //  */
        // static std::shared_ptr<okapi::Logger> getLogger()
        // {
        //     return okapi::Logger::getDefaultLogger();
        // }

    private:
        Logger() = delete;

        inline static const std::string LOG_TERMINAL = "/ser/sout";
        static constexpr bool LOG_TO_DISPLAY = false;
        static constexpr bool LOG_TO_SD = false;

        inline static std::ofstream logFileStream = std::ofstream();
    };
}