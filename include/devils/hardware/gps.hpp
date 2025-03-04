#pragma once
#include "pros/gps.hpp"
#include "../utils/logger.hpp"
#include "../geometry/pose.hpp"
#include "../geometry/units.hpp"
#include "../geometry/polygon.hpp"
#include "../odom/odomSource.hpp"
#include "structs/hardwareBase.hpp"

namespace devils
{
    /**
     * Represents a Vex V5 GPS
     */
    class GPS : public OdomSource, public Runnable, private HardwareBase
    {
    public:
        /**
         * Initializes a Vex V5 GPS as an odometry source.
         * Remember to call `GPS::setPosition` to set the initial position before the GPS locks on.
         * Also call `GPS::setOffset` to set the GPS's position relative to the center of the robot.
         * @param gpsPort The port of the GPS
         */
        GPS(std::string name, uint8_t gpsPort)
            : HardwareBase(name, "GPS", gpsPort),
              gps(gpsPort, 0, 0, 0, 0, 0)
        {
            if (errno != 0)
                reportFault("Invalid port");
            gps.set_data_rate(20);
        }

        /**
         * Updates the odometry with the latest GPS data
         */
        void onUpdate() override
        {
            double gpsX = gps.get_position_x();
            double gpsY = gps.get_position_y();
            double gpsHeading = gps.get_heading();

            if (gpsX == PROS_ERR_F || gpsY == PROS_ERR_F || gpsHeading == PROS_ERR_F)
            {
                reportFault("Get GPS position failed");
                return;
            }

            // Convert Units
            gpsX = Units::metersToIn(gpsX);
            gpsY = -Units::metersToIn(gpsY);
            gpsHeading = Units::normalizeRadians(Units::degToRad(gpsHeading) - GPS_ROTATION_OFFSET - rotationalOffset);

            // Check Calibrating
            if (isCalibrating())
                return;

            // Check Within Bounds
            if (gpsX < -MAX_GPS_X || gpsX > MAX_GPS_X || gpsY < -MAX_GPS_Y || gpsY > MAX_GPS_Y)
            {
                reportFault("GPS out of bounds");
                return;
            }

            // Update Pose
            currentPose.x = gpsX;
            currentPose.y = gpsY;
            currentPose.rotation = gpsHeading;
        }

        /**
         * Restarts the calibration timer
         */
        void reset()
        {
            calibrationStartTime = -1;
        }

        /**
         * Gets the current pose of the robot since the last `GPS::update`
         * @return The current pose of the robot
         */
        Pose &getPose() override
        {
            return currentPose;
        }

        /**
         * Sets the current pose of the robot.
         * This value is overriden when the GPS locks on to the field strip.
         * @param pose The pose to set the robot to
         * @return The current pose of the robot
         */
        void setPose(Pose &pose) override
        {
            currentPose = pose;

            double gpsX = Units::inToMeters(pose.x);
            double gpsY = Units::inToMeters(pose.y);
            double gpsYaw = Units::degToRad(pose.rotation) + GPS_ROTATION_OFFSET + rotationalOffset;

            int32_t status = gps.set_position(
                gpsX,
                gpsY,
                gpsYaw);

            if (status != 1)
                reportFault("Set GPS position failed");
        }

        /**
         * Sets the GPS's position relative to the center of the robot.
         * @param x The x offset of the GPS from the center of the robot in inches
         * @param y The y offset of the GPS from the center of the robot in inches
         * @param rotation The rotation offset of the GPS from the center of the robot in radians
         */
        void setOffset(double x, double y, double rotation)
        {
            int32_t result = gps.set_offset(
                Units::inToMeters(x),
                Units::inToMeters(y));
            rotationalOffset = rotation;

            if (result != 1)
                reportFault("Set GPS offset failed");
        }

        /**
         * Checks if the GPS is calibrating. GPS takes a few seconds to lock on to the field strip.
         * @return Whether the GPS is calibrating
         */
        bool isCalibrating()
        {
            if (calibrationStartTime < 0)
                calibrationStartTime = pros::millis();
            return (pros::millis() - calibrationStartTime) < CALIBRATION_TIME;
        }

    protected:
        void serialize() override
        {
            ntX.set(currentPose.x);
            ntY.set(currentPose.y);
            ntRotation.set(currentPose.rotation);

            if (!gps.is_installed())
                reportFault("Disconnected");
        }

    private:
        static constexpr int CALIBRATION_TIME = 7000;             // ms
        static constexpr double GPS_ROTATION_OFFSET = M_PI * 0.5; // PROS defaults to north as 0 degrees
        static constexpr double MAX_GPS_X = 72;
        static constexpr double MAX_GPS_Y = 72;

        NTValue<double> ntX = ntGroup.makeValue("x", 0.0);
        NTValue<double> ntY = ntGroup.makeValue("y", 0.0);
        NTValue<double> ntRotation = ntGroup.makeValue("rotation", 0.0);

        pros::Gps gps;
        Pose currentPose = Pose(0, 0, 0);
        double rotationalOffset = 0;
        int calibrationStartTime = -1;
    };
}