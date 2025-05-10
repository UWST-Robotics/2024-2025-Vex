
#include <pros/adi.hpp>

#pragma once

namespace devils
{
    class LEDStrip
    {
    public:
        LEDStrip(char* name, int pin, int length) : strip(pin, length)
        {
            this->name = name;
        }

        void set_color(uint32_t color)
        {
            strip.set_all(color);
        }

    private:
        char* name;
        int pin;
        int length;
        pros::adi::Led strip;
    };
}