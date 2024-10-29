#pragma once
#include "networkObject.hpp"
#include "../odom/odomSource.hpp"
#include "../network/networkTables.hpp"
#include "pros/misc.hpp"

namespace devils
{
    class NetworkRobotState : private INetworkObject
    {
    public:
        void serialize() override
        {
            // Get Prefix
            std::string networkTableKey = "_robot";

            // Battery
            NetworkTables::UpdateValue(networkTableKey + "/batteryVoltage", std::to_string(pros::battery::get_voltage()));
            NetworkTables::UpdateValue(networkTableKey + "/batteryCurrent", std::to_string(pros::battery::get_current()));
            NetworkTables::UpdateValue(networkTableKey + "/batteryTemperature", std::to_string(pros::battery::get_temperature()));

            // Competition
            NetworkTables::UpdateValue(networkTableKey + "/competitionStatus", std::to_string(pros::competition::get_status()));
            NetworkTables::UpdateValue(networkTableKey + "/isCompetition", std::to_string(pros::competition::is_connected()));

            // Controller
            // Disabled due to performance overflow
            // NetworkTables::UpdateValue(networkTableKey + "/mainControllerConnected", std::to_string(mainController.is_connected()));
            // NetworkTables::UpdateValue(networkTableKey + "/mainControllerBatteryVoltage", std::to_string(mainController.get_battery_capacity()));
            // NetworkTables::UpdateValue(networkTableKey + "/mainControllerBatteryLevel", std::to_string(mainController.get_battery_level()));

            // NetworkTables::UpdateValue(networkTableKey + "/partnerControllerConnected", std::to_string(partnerController.is_connected()));
            // NetworkTables::UpdateValue(networkTableKey + "/partnerControllerBatteryVoltage", std::to_string(partnerController.get_battery_capacity()));
            // NetworkTables::UpdateValue(networkTableKey + "/partnerControllerBatteryLevel", std::to_string(partnerController.get_battery_level()));

            // SD Card
            NetworkTables::UpdateValue(networkTableKey + "/isSDCardInstalled", std::to_string(pros::usd::is_installed()));
        }

    private:
        pros::Controller mainController = pros::Controller(pros::E_CONTROLLER_MASTER);
        pros::Controller partnerController = pros::Controller(pros::E_CONTROLLER_PARTNER);
    };
}