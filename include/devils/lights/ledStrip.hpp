#pragma once
#include "pros/adi.hpp"

#pragma once

namespace devils
{
    enum StripAction {
        PULSE_BLUE = 50,
        WAVE_BLUE = 250,
        WAVE_GREEN = 450,
        PULSE_GREEN = 550,
        PULSE_RED = 650,

    };

    class LEDStrip
    {
    public:
        LEDStrip(int port) : ledStrip(port)
        {
            this->pulseBlue();
        };

        void pulseBlue()
        {
            ledStrip.set_value(PULSE_BLUE);
        }
        void waveBlue()
        {
            ledStrip.set_value(WAVE_BLUE);
        }
        void pulseRed()
        {
            ledStrip.set_value(PULSE_GREEN);
        }
        void waveGreen()
        {
            ledStrip.set_value(WAVE_GREEN);
        }
        void pulseGreen()
        {
            ledStrip.set_value(PULSE_RED);
        }
    private:
        pros::ADIMotor ledStrip;
    };
}