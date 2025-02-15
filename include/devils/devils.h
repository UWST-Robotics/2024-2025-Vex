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
#include "hardware/gyro.hpp"
#include "hardware/inertialSensor.hpp"
#include "hardware/inertialSensorGroup.hpp"
#include "hardware/opticalSensor.hpp"
#include "hardware/visionSensor.hpp"
#include "hardware/adiPneumatic.hpp"
#include "hardware/adiPneumaticGroup.hpp"

// Odom
#include "odom/odomSource.hpp"
#include "odom/complementaryFilterOdom.hpp"
#include "odom/differentialWheelOdom.hpp"
#include "odom/tankChassisOdom.hpp"
#include "odom/perpendicularSensorOdom.hpp"
#include "odom/parallelSensorOdom.hpp"

// Pros
#include "api.h"

// Robot
#include "utils/robot.hpp"

// Utils
#include "utils/joystickCurve.hpp"
#include "utils/timer.hpp"

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

// AutoSteps
#include "autoSteps/common/autoStep.hpp"
#include "autoSteps/common/autoStepList.hpp"
#include "autoSteps/common/autoBuilder.hpp"
#include "autoSteps/steps/autoTimeoutStep.hpp"
#include "autoSteps/steps/autoDriveTimeStep.hpp"
#include "autoSteps/steps/autoDriveToStep.hpp"
#include "autoSteps/steps/autoDriveStep.hpp"
#include "autoSteps/steps/autoRotateStep.hpp"
#include "autoSteps/steps/autoRotateToStep.hpp"
#include "autoSteps/steps/autoPauseStep.hpp"
#include "autoSteps/steps/autoJumpToStep.hpp"
#include "autoSteps/steps/autoBranchStep.hpp"

// Display
#include "display/eyesRenderer.hpp"

// Utils
#include "utils/ntOdom.hpp"

// VEXBridge
#include "../vexbridge/vexbridge.h"