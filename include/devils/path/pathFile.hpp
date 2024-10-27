#pragma once
#include <string>
#include <vector>
#include "../geometry/pose.hpp"

namespace devils
{
    /**
     * A struct representing an event at a point in the robot path.
     */
    struct PathEvent
    {
        PathEvent(std::string name, std::string params)
            : id(std::rand()),
              name(name),
              params(params)
        {
        }

        /// @brief Random ID created at runtime, unique to each event
        int id = 0;
        /// @brief The name of the event
        std::string name;
        /// @brief The parameters of the event
        std::string params;

        /**
         * Converts the event to a string.
         * @return The event as a string.
         */
        std::string toString() const
        {
            return "Event: " + name + " (" + params + ")";
        }
    };

    /**
     * A vector of `PathEvent` structs.
     */
    typedef std::vector<PathEvent> PathEvents;

    /**
     * A struct representing a control point in the robot path.
     */
    struct ControlPoint : public Pose
    {
        /// @brief The entry delta of the robot in inches
        double enterDelta = 0;
        /// @brief The exit delta of the robot in inches
        double exitDelta = 0;
        /// @brief Whether the robot is reversed at this point
        bool isReversed = false;
        /// @brief The events at this point
        PathEvents events = {};

        /**
         * Constructs a path point with all values set to 0.
         */
        ControlPoint() : Pose(0, 0, 0) {}

        /**
         * Constructs a path point from a `Pose`.
         * @param pose The pose to construct the path point from.
         */
        ControlPoint(const Pose &pose) : Pose(pose) {}

        /**
         * Copy constructor.
         * @param pathPoint The path point to copy.
         */
        ControlPoint(const ControlPoint &pathPoint) : Pose(pathPoint.x, pathPoint.y, pathPoint.rotation),
                                                      enterDelta(pathPoint.enterDelta),
                                                      exitDelta(pathPoint.exitDelta),
                                                      isReversed(pathPoint.isReversed),
                                                      events(pathPoint.events)
        {
        }

        /**
         * Converts the point to a string.
         * @return The point as a string.
         */
        std::string toString() const
        {
            std::string str = "Point: (" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(rotation) + ")[" + std::to_string(enterDelta) + ", " + std::to_string(exitDelta) + "] R=" + std::to_string(isReversed) + "\n";
            for (PathEvent event : events)
                str += "    " + event.toString() + "\n";
            return str;
        }
    };

    /**
     * A vector of `ControlPoint` structs.
     */
    typedef std::vector<ControlPoint> ControlPoints;

    /**
     * A struct representing a robot path.
     */
    struct PathFile
    {
        /// @brief The version of the path file
        int version = 0;
        /// @brief The control points in the path
        ControlPoints points;

        /**
         * Converts the path to a string.
         * @return The path as a string.
         */
        std::string toString() const
        {
            std::string str = "PathFile " + std::to_string(version) + "\n====================\n";
            for (ControlPoint point : points)
                str += point.toString();
            str += "====================";
            return str;
        }
    };
}