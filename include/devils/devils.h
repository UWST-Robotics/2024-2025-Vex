#pragma once

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

/*
 *      Headers defined in this file are accessible from anywhere in the project.
 */

// Autonomous
#include "control/pursuitController.hpp"
#include "control/linearController.hpp"
#include "control/findController.hpp"
#include "control/directController.hpp"
#include "control/timeController.hpp"
#include "control/chaseController.hpp"
#include "control/collectionController.hpp"
#include "control/controllerList.hpp"

// Chassis
#include "chassis/chassis.hpp"
#include "chassis/tankChassis.hpp"
#include "chassis/dummyChassis.hpp"

// Display
// DEPRICATED in favor of BlueBox
// #include "display/display.hpp"
// #include "display/odomRenderer.hpp"
// #include "display/pathRenderer.hpp"
// #include "display/controlRenderer.hpp"
// #include "display/displayUtils.hpp"
// #include "display/fieldRenderer.hpp"
// #include "display/gridRenderer.hpp"
// #include "display/polygonRenderer.hpp"
// #include "display/gameObjectRenderer.hpp"
// #include "display/autoPickerRenderer.hpp"
// #include "display/occupancyRenderer.hpp"
// #include "display/statsRenderer.hpp"
// #include "display/poseRenderer.hpp"

// Hardware
#include "hardware/gps.hpp"
#include "hardware/imu.hpp"
#include "hardware/opticalSensor.hpp"
#include "hardware/visionSensor.hpp"
#include "hardware/scuffPneumatic.hpp"
#include "hardware/scuffPneumaticGroup.hpp"

// Odom
#include "odom/odomSource.hpp"
#include "odom/complementaryFilterOdom.hpp"
#include "odom/trackingWheelOdom.hpp"
#include "odom/differentialWheelOdom.hpp"
#include "odom/transformOdom.hpp"

// Path
#include "path/pathGenerator.hpp"
#include "path/pathFinder.hpp"
#include "path/occupancyGrid.hpp"
#include "path/occupancyFileReader.hpp"

// Game Object
#include "gameobject/gameObject.hpp"
#include "gameobject/gameObjectManager.hpp"

// Geometry
#include "geometry/perspectiveFactory.hpp"

// Pros
#include "api.h"

// Robot
#include "utils/robot.hpp"

// Utils
#include "utils/joystickCurve.hpp"
#include "utils/pid.hpp"
#include "utils/eventTimer.hpp"
