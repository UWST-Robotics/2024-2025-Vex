#include "stdint.h"

#ifndef DEVILS_SDK_EXTENSIONS_H
#define DEVILS_SDK_EXTENSIONS_H

#define COMPETITION_DISABLED (1 << 0)
#define COMPETITION_AUTONOMOUS (1 << 1)
#define COMPETITION_CONNECTED (1 << 2)

extern "C"
{
    // Methods defined here are part of the closed-source VEX V5 SDK
    // Note: Use at your own risk, as these methods don't have the same level of safety as PROS methods
    // uint32_t vexSystemVersion();
    // uint32_t vexSdkVersion();

    // int32_t vexBatteryVoltageGet();
    // int32_t vexBatteryCurrentGet();
    // double vexBatteryTemperatureGet();
    // double vexBatteryCapacityGet();

    // uint32_t vexCompetitionStatus();
}

#endif // DEVILS_SDK_EXTENSIONS_H