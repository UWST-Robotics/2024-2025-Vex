#pragma once

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

/*
 *      Headers defined in this file are accessible from anywhere in the project.
 */

// Chassis
#include "chassis/chassisBase.hpp"
#include "chassis/tankChassis.hpp"
#include "chassis/dummyChassis.hpp"

// Hardware
#include "hardware/gps.hpp"
#include "hardware/imu.hpp"
#include "hardware/opticalSensor.hpp"
#include "hardware/visionSensor.hpp"
#include "hardware/adiPneumatic.hpp"
#include "hardware/adiPneumaticGroup.hpp"

// Odom
#include "odom/odomSource.hpp"
#include "odom/complementaryFilterOdom.hpp"
#include "odom/trackingWheelOdom.hpp"
#include "odom/differentialWheelOdom.hpp"
#include "odom/tankChassisOdom.hpp"

// Geometry
#include "geometry/perspectiveFactory.hpp"

// Pros
#include "api.h"

// Robot
#include "utils/robot.hpp"

// Utils
#include "utils/joystickCurve.hpp"
#include "utils/eventTimer.hpp"

// Path
#include "path/path.hpp"
#include "path/linearPath.hpp"
#include "path/splinePath.hpp"
#include "path/structs/splinePose.hpp"
#include "path/autoDevilDeserializer.hpp"

// Trajectory
#include "trajectory/trajectory.hpp"
#include "trajectory/trajectoryGenerator.hpp"
#include "trajectory/structs/trajectoryConstraints.hpp"
#include "trajectory/structs/trajectoryState.hpp"

// Network
#include "nt/networkTables.hpp"
#include "nt/ntService.hpp"
#include "nt/ntObjectBase.hpp"
#include "nt/objects/ntOdom.hpp"
#include "nt/objects/ntPath.hpp"
#include "nt/objects/ntHardware.hpp"

// AutoSteps
#include "autoSteps/common/autoStep.hpp"
#include "autoSteps/common/autoStepList.hpp"
#include "autoSteps/autoDriveStep.hpp"
#include "autoSteps/autoDriveToStep.hpp"
#include "autoSteps/autoRotateStep.hpp"
#include "autoSteps/autoRotateToStep.hpp"
#include "autoSteps/autoPauseStep.hpp"
#include "autoSteps/autoJumpToStep.hpp"