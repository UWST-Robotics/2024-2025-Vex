#pragma once
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/llemu.hpp"
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <map>

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
         * The log level (INFO, WARN, ERROR, DEBUG).
         */
        enum LogLevel
        {
            INFO,
            WARN,
            ERROR,
            DEBUG
        };

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

            // Open Log File
            if (LOG_TO_SD)
            {
                if (!isSDInserted())
                {
                    warn("Logger: SD card is not installed!");
                    return;
                }

                // Set up the log file
                std::string path = getLogFilePath();
                logFileStream.open(path);
                if (logFileStream.is_open())
                    info("Logger: Opened log file at " + path);
                else
                    warn("Logger: Failed to open log file at " + path);
            }
        }

        /**
         * Logs a message to the terminal, display, SD card, and network.
         * @param message The message to log.
         * @param level The log level.
         */
        static void log(std::string message, LogLevel level)
        {
            // Log to terminal
            if (LOG_TO_SERIAL)
                logToSerial(message);

            // Log to display
            if (LOG_TO_DISPLAY)
                logToLCD(message);

            // Log to SD card
            if (LOG_TO_SD)
                logToSD(message);
        }

        /**
         * Logs an info message.
         * @param message The message to log.
         */
        static void info(std::string message)
        {
            log(message, LogLevel::INFO);
        }

        /**
         * Logs a warning message.
         * @param message The message to log.
         */
        static void warn(std::string message)
        {
            log(message, LogLevel::WARN);
        }

        /**
         * Logs an error message.
         * @param message The message to log.
         */
        static void error(std::string message)
        {
            log(message, LogLevel::ERROR);
        }

        /**
         * Logs a debug message.
         * @param message The message to log.
         */
        static void debug(std::string message)
        {
            log(message, LogLevel::DEBUG);
        }

        /**
         * Logs a message to the SD card.
         * @param message The message to log.
         */
        static void logToSD(std::string message)
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
        static void logToLCD(std::string message)
        {
            static int line = 0;
            pros::lcd::set_text(line, message);
            line = (line + 1) % 8;
        }

        /**
         * Logs a message to the serial port.
         * @param message The message to log.
         */
        static void logToSerial(std::string message)
        {
            std::cout << message << std::endl;
        }

    private:
        /**
         * Gets a file path for a new log file. Increments the index until a file that doesn't exist is found.
         * @return The file path for a new log file.
         */
        static std::string getLogFilePath()
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

        Logger() = delete;

        inline static const std::string TERMINAL_PATH = "/ser/sout";
        static constexpr bool LOG_TO_DISPLAY = false;
        static constexpr bool LOG_TO_SD = false;
        static constexpr bool LOG_TO_SERIAL = true;

        inline static std::ofstream logFileStream = std::ofstream();
    };
}