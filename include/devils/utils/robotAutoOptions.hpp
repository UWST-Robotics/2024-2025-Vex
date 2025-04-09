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

    struct Routine{
        unsigned int id;
        std::string displayName;
        bool requiresAllianceColor = false;
    };

    struct RobotAutoOptions
    {
        AllianceColor allianceColor = NONE_ALLIANCE;
        Routine routine = {0, "None", false};
    };

    

}