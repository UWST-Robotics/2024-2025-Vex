#pragma once
#include <string>
#include <vector>

namespace devils
{
    /**
     * A class containing a varitet of string utilities.
     */
    class StringUtils
    {
    public:
        /**
         * Splits a string by a delimiter.
         * @param str The string to split.
         * @param delimiter The delimiter to split by.
         */
        static std::vector<std::string> split(std::string inputText, char delimiter)
        {
            std::vector<std::string> result;
            std::string buffer = "";
            // Iterate through characters
            for (int i = 0; i < inputText.length(); i++)
            {
                // Flush current string if matches delimiter
                if (inputText[i] == delimiter)
                {
                    result.push_back(buffer);
                    buffer = "";
                }
                // Otherwise, add to buffer
                else
                {
                    buffer += inputText[i];
                }
            }
            // Flush last string
            result.push_back(buffer);
            return result;
        }

        static std::string replace(std::string inputText, char character, char replacementChar)
        {
            std::string result = "";
            for (int i = 0; i < inputText.length(); i++)
            {
                if (inputText[i] == character)
                    result += replacementChar;
                else
                    result += inputText[i];
            }
            return result;
        }

    private:
        // Private constructor to prevent instantiation
        StringUtils() = delete;
    };
}