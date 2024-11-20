#pragma once

#include "pros/imu.hpp"
#include "pros/error.h"
#include "../utils/logger.hpp"
#include "../nt/objects/ntHardware.hpp"
#include "../geometry/units.hpp"
#include "../geometry/vector3.hpp"
#include "../odom/odomSource.hpp"
#include <string>

namespace devils
{
    /**
     * Represents a V5 inertial measurement unit.
     */
    class IMU : public OdomSource, private NTHardware
    {
    public:
        /**
         * Creates a new IMU.
         * Remember to calibrate the IMU before use.
         * @param name The name of the IMU (for logging purposes)
         * @param port The port of the IMU (from 1 to 21)
         */
        IMU(std::string name, uint8_t port)
            : NTHardware(name, "IMU", port),
              imu(port)
        {
            if (errno != 0)
                reportFault("Invalid port");
        }

        /**
         * Converts heading to an `OdomSource` at (0, 0).
         * @param heading The heading to convert to an `OdomSource`.
         */
        Pose &getPose() override
        {
            odomPose.rotation = getHeading();
            return odomPose;
        }

        /**
         * Sets the heading of the IMU from a given `Pose`.
         * @param pose The pose to set the IMU to. Only uses `Pose.rotation`.
         */
        void setPose(Pose &pose) override
        {
            setHeading(pose.rotation);
        }

        /**
         * Gets the current acceleration of the IMU in inches per second squared.
         * @return The current acceleration of the IMU in inches per second squared.
         */
        Vector3 getAccel()
        {
            auto accel = imu.get_accel();
            if (accel.x == PROS_ERR_F)
            {
                reportFault("Get IMU acceleration failed");
                return Vector3(0, 0, 0);
            }
            return Vector3(
                Units::metersToIn(accel.x),
                Units::metersToIn(accel.y),
                Units::metersToIn(accel.z));
        }

        /**
         * Gets the current heading of the IMU in radians, unbounded.
         * @return The current heading of the IMU in radians or 0 if the operation failed.
         */
        double getHeading()
        {
            double heading = imu.get_rotation();
            if (heading == PROS_ERR_F)
            {
                reportFault("Get IMU heading failed");
                return 0;
            }

            // Apply scale/offset
            return Units::degToRad(heading) * headingScale + headingOffset;
        }

        /**
         * Gets the current pitch of the IMU in radians.
         * @return The current pitch of the IMU in radians or 0 if the operation failed.
         */
        double getPitch()
        {
            double pitch = imu.get_pitch();
            if (pitch == PROS_ERR_F)
            {
                reportFault("Get IMU pitch failed");
                return 0;
            }
            return Units::degToRad(pitch);
        }

        /**
         * Gets the current roll of the IMU in radians.
         * @return The current roll of the IMU in radians or 0 if the operation failed.
         */
        double getRoll()
        {
            double roll = imu.get_roll();
            if (roll == PROS_ERR_F)
            {
                reportFault("Get IMU roll failed");
                return 0;
            }
            return Units::degToRad(roll);
        }

        /**
         * Gets the current yaw of the IMU in radians.
         * @return The current yaw of the IMU in radians or 0 if the operation failed.
         */
        double getYaw()
        {
            double yaw = imu.get_yaw();
            if (yaw == PROS_ERR_F)
            {
                reportFault("Get IMU yaw failed");
                return 0;
            }
            return Units::degToRad(yaw);
        }

        /**
         * Sets the current heading of the IMU in radians.
         * @param heading The heading to set the IMU to in radians.
         */
        void setHeading(double heading)
        {
            headingOffset = heading - getHeading();
        }

        /**
         * Scales the heading by a given factor.
         * Used to fix consistent heading drift after rotation.
         * @param scale The scale to multiply the heading by.
         */
        void setHeadingScale(double scale)
        {
            headingScale = scale;
        }

        /**
         * Calibrates the IMU. Robot should be still during calibration.
         * Run `waitUntilCalibrated` to wait until calibration is finished.
         */
        void calibrate()
        {
            imu.reset(false);
        }

        /**
         * Waits until the IMU is finished calibrating.
         * Should be ran to avoid movement during calibration.
         */
        void waitUntilCalibrated()
        {
            while (imu.is_calibrating() && imu.is_installed())
                pros::delay(20);
        }

    protected:
        void serializeHardware(std::string &ntPrefix) override
        {
            auto acceleration = getAccel();

            NetworkTables::updateDoubleValue(ntPrefix + "/heading", Units::radToDeg(getHeading()));
            NetworkTables::updateDoubleValue(ntPrefix + "/pitch", Units::radToDeg(getPitch()));
            NetworkTables::updateDoubleValue(ntPrefix + "/roll", Units::radToDeg(getRoll()));
            NetworkTables::updateDoubleValue(ntPrefix + "/yaw", Units::radToDeg(getYaw()));
            NetworkTables::updateDoubleValue(ntPrefix + "/accelX", acceleration.x);
            NetworkTables::updateDoubleValue(ntPrefix + "/accelY", acceleration.y);
            NetworkTables::updateDoubleValue(ntPrefix + "/accelZ", acceleration.z);
        }

        void checkHealth() override
        {
            // Status Check
            pros::ImuStatus imuStatus = imu.get_status();
            isCalibrating = imuStatus == pros::ImuStatus::calibrating;
            isErrored = imuStatus == pros::ImuStatus::error;

            // Check if IMU is Connected
            isConnected = imu.is_installed();

            // Report Fault
            if (!isConnected)
                reportFault("Disconnected");
            else if (isCalibrating)
                reportFault("Calibrating");
            else if (isErrored)
                reportFault("Unknown Error");
            else
                clearFaults();
        }

    private:
        double headingScale = 1;
        double headingOffset = 0;
        bool isCalibrating = false;
        bool isErrored = false;
        bool isConnected = false;

        pros::IMU imu;
        Pose odomPose;
    };
}