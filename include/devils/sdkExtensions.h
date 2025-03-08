#include "stdint.h"

#ifndef DEVILS_SDK_EXTENSIONS_H
#define DEVILS_SDK_EXTENSIONS_H

/// @brief Structure to hold the object data from the AI Vision Sensor
typedef struct __attribute((packed)) _V5_DeviceAiVisionObject
{
    uint8_t id;   /// object color/id
    uint8_t type; /// object type
    union
    {
        struct
        {
            uint16_t xoffset; /// left side of object
            uint16_t yoffset; /// top of object
            uint16_t width;   /// width of object
            uint16_t height;  /// height of object
            uint16_t angle;   /// angle of CC object in 0.1 deg units
        } color;

        struct
        {
            int16_t x0; /// apriltag coords
            int16_t y0; ///
            int16_t x1; ///
            int16_t y1; ///
            int16_t x2; ///
            int16_t y2; ///
            int16_t x3; ///
            int16_t y3; ///
        } tag;

        struct
        {
            uint16_t xoffset; /// left side of object
            uint16_t yoffset; /// top of object
            uint16_t width;   /// width of object
            uint16_t height;  /// height of object
            uint16_t score;   /// confidence score
        } model;
    } object;
} V5_DeviceAiVisionObject;

extern "C"
{
    // Methods defined here are part of the closed-source VEX V5 SDK
    int32_t vexDeviceAiVisionObjectCountGet(uint16_t port);
    int32_t vexDeviceAiVisionObjectGet(uint16_t device, uint32_t indexObj, V5_DeviceAiVisionObject *pObject);
}

#endif // DEVILS_SDK_EXTENSIONS_H