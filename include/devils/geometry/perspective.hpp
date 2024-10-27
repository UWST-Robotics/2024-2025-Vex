#pragma once
#include "../utils/logger.hpp"
#include "Vector2.hpp"
#include "units.hpp"
#include "pose.hpp"
#include <cmath>

// https://www.vexforum.com/t/eigen-integration-issue/61474/7
#undef __ARM_NEON__
#undef __ARM_NEON
#include "Eigen/Dense"

namespace devils
{

    /**
     * A class representing a perspective transformation from a camera to a flat plane.
     */
    class Perspective
    {
    public:
        /**
         * Generates a new perspective transform from camera calibration parameters.
         * @param verticalFOV The vertical field of view of the camera in degrees.
         * @param nearClip The distance to the near clipping plane in inches.
         * @param farClip The distance to the far clipping plane in inches.
         * @param width The width of the camera in pixels.
         * @param height The height of the camera in pixels.
         * @param cameraHeight The height of the camera above the ground in inches.
         * @param cameraPitch The pitch of the camera in degrees.
         */
        Perspective(double verticalFOV, double nearClip, double farClip, double width, double height, double cameraHeight, double cameraPitch)
        {
            // TODO: Implement perspective transform
            auto viewMat = _createViewTransformMatrix(cameraHeight, Units::degToRad(cameraPitch));
            auto projectionMat = _calculateProjectionMatrix(Units::degToRad(verticalFOV), nearClip, farClip, width / height);

            transformationMatrix = projectionMat * viewMat;
        }
        Perspective(const Perspective &other)
        {
            transformationMatrix = other.transformationMatrix;
        }

        /**
         * Calculate the view transformation matrix from camera position and pitch.
         * @param height The height of the camera above the ground in inches.
         * @param pitch The pitch of the camera in radians.
         */
        Eigen::Matrix4d _createViewTransformMatrix(double height, double pitch)
        {
            Eigen::Matrix4d viewTransform = Eigen::Matrix4d::Identity();
            viewTransform(1, 3) = -height;
            viewTransform(0, 0) = cos(pitch);
            viewTransform(0, 2) = sin(pitch);
            viewTransform(2, 0) = -sin(pitch);
            viewTransform(2, 2) = cos(pitch);
            return viewTransform;
        }

        /**
         * Calculates the projection transformation matrix from camera calibration parameters.
         * @param verticalFOV The vertical field of view of the camera in radians.
         * @param nearClip The distance to the near clipping plane in inches.
         * @param farClip The distance to the far clipping plane in inches.
         * @param aspectRatio The aspect ratio of the camera.
         * @return The transformation matrix.
         */
        Eigen::Matrix4d _calculateProjectionMatrix(
            double verticalFOV,
            double nearClip,
            double farClip,
            double aspectRatio)
        {
            // Calculate Frustum Planes
            double halfFOV = verticalFOV / 2;
            double top = nearClip * tan(halfFOV);
            double bottom = -top;
            double right = top * aspectRatio;
            double left = -right;

            // Calculate Frustum Points
            double sx = 2 * nearClip / (right - left);
            double sy = 2 * nearClip / (top - bottom);

            double c2 = -(farClip + nearClip) / (farClip - nearClip);
            double c1 = 2 * farClip * nearClip / (nearClip - farClip);

            double tx = -nearClip * (left + right) / (right - left);
            double ty = -nearClip * (top + bottom) / (top - bottom);

            // Create Projection Matrix
            Eigen::Matrix4d projectionMatrix = Eigen::Matrix4d::Zero();
            projectionMatrix(0, 0) = sx;
            projectionMatrix(1, 1) = sy;
            projectionMatrix(2, 2) = c2;
            projectionMatrix(2, 3) = -1;
            projectionMatrix(3, 2) = c1;
            projectionMatrix(0, 3) = tx;
            projectionMatrix(1, 3) = ty;

            return projectionMatrix;
        }

        /**
         * Transforms a point from screen space to world space.
         * @param point The point to transform.
         * @return The transformed point.
         */
        Vector2 screenToWorld(Vector2 point)
        {
            Eigen::Vector4d screenPoint(point.x, point.y, 0, 1);
            Eigen::Vector4d worldPoint = transformationMatrix.inverse() * screenPoint;
            return Vector2{worldPoint(0) / worldPoint(3), worldPoint(1) / worldPoint(3)};
        }

        /**
         * Transforms a point from world space to screen space.
         * @param point The point to transform.
         * @return The transformed point.
         */
        Vector2 worldToScreen(Vector2 point)
        {
            Eigen::Vector4d worldPoint(point.x, point.y, 0, 1);
            Eigen::Vector4d screenPoint = transformationMatrix * worldPoint;
            return Vector2{screenPoint(0) / screenPoint(3), screenPoint(1) / screenPoint(3)};
        }

    private:
        Eigen::Matrix4d transformationMatrix;
    };
}