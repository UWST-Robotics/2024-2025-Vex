#pragma once

#include <cstdint>
#include <string>
#include "pros/llemu.hpp"

namespace bluebox
{
    /**
     * Represents a logger that logs messages to the LCD and console.
     */
    class NTLogger
    {
    public:
        /**
         * Logs a warning message to the console.
         * @param message The message to log.
         */
        static void logWarning(std::string message)
        {
            log("WARNING: " + message);
        }

        /**
         * Logs a message to the console.
         * @param message The message to log.
         */
        static void log(std::string message)
        {
            if (PRINT_TO_LCD)
                pros::lcd::print(0, message.c_str());
            if (PRINT_TO_CONSOLE)
                printf("%s\n", message.c_str());
        }

    private:
        NTLogger() = delete;
        NTLogger(const NTLogger &) = delete;
        NTLogger &operator=(const NTLogger &) = delete;

        static constexpr bool PRINT_TO_LCD = true;
        static constexpr bool PRINT_TO_CONSOLE = true;
    };
};