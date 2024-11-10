#pragma once
#include "networkObject.hpp"
#include "../odom/odomSource.hpp"
#include "../network/networkTables.hpp"
#include "pros/misc.hpp"
#include "devils/sdkExtensions.h"

namespace devils
{
    class NetworkRobotState : private INetworkObject
    {
    public:
        NetworkRobotState()
        {
            setSerializationRate(500);
            systemVersion = getSystemVersion();
            sdkVersion = getSDKVersion();
        }

        void serialize() override
        {
            // Send Heartbeat
            NetworkTables::SendHeartbeat();

            // Get Prefix
            std::string networkTableKey = "_hardware/robot";

            NetworkTables::UpdateValue(networkTableKey + "/name", "Robot");
            NetworkTables::UpdateValue(networkTableKey + "/type", "VEX V5");

            // Version
            NetworkTables::UpdateValue(networkTableKey + "/prosVersion", PROS_VERSION_STRING);
            NetworkTables::UpdateValue(networkTableKey + "/systemVersion", systemVersion);
            NetworkTables::UpdateValue(networkTableKey + "/sdkVersion", sdkVersion);

            // Battery
            NetworkTables::UpdateValue(networkTableKey + "/batteryVoltage", std::to_string(vexBatteryVoltageGet()));
            NetworkTables::UpdateValue(networkTableKey + "/batteryCurrent", std::to_string(vexBatteryCurrentGet()));
            NetworkTables::UpdateValue(networkTableKey + "/batteryTemperature", std::to_string(vexBatteryTemperatureGet()));

            // Competition
            uint32_t competitionStatus = vexCompetitionStatus();
            NetworkTables::UpdateValue(networkTableKey + "/isAutonomous", std::to_string(competitionStatus & COMPETITION_AUTONOMOUS ? true : false));
            NetworkTables::UpdateValue(networkTableKey + "/isDisabled", std::to_string(competitionStatus & COMPETITION_DISABLED ? true : false));
            NetworkTables::UpdateValue(networkTableKey + "/isConnected", std::to_string(competitionStatus & COMPETITION_CONNECTED ? true : false));
        }

    private:
        std::string getSystemVersion()
        {
            // Get System Version from VEX
            uint8_t version[4];
            uint32_t *versionPointer = (uint32_t *)version;
            *versionPointer = vexSystemVersion();

            // Format Version
            return std::to_string(version[3]) + "." + std::to_string(version[2]) + "." + std::to_string(version[1]) + "-" + std::to_string(version[0]);
        }

        std::string getSDKVersion()
        {
            // Get System Version from VEX
            uint8_t version[4];
            uint32_t *versionPointer = (uint32_t *)version;
            *versionPointer = vexSdkVersion();

            return std::to_string(version[3]) + "." + std::to_string(version[2]) + "." + std::to_string(version[1]) + "-" + std::to_string(version[0]);
        }

        std::string systemVersion;
        std::string sdkVersion;
    };
}