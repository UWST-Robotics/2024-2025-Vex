#pragma once

#include <string>

namespace devils
{
    enum AllianceColor
    {
        RED_ALLIANCE = 0,
        BLUE_ALLIANCE = 1,
        NONE_ALLIANCE = 2
    };
    struct RobotAutoOptions
    {
        AllianceColor allianceColor = NONE_ALLIANCE;
        std::string routine = "NONE";
    };

}