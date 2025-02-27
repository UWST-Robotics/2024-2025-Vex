// #pragma once

// #include "pros/rtos.hpp"

// namespace devils
// {
//     /**
//      * Represents the parameters of a Ramp controller.
//      */
//     struct RampParams
//     {
//         double maxVelocity;
//     };

//     /**
//      * Represents a controller that clamps the rate of change of a value.
//      */
//     class RampController
//     {
//     public:
//         /**
//          * Creates a new ramp controller.
//          * @param maxAccel The maximum acceleration of the controller.
//          * @param maxDecel The maximum deceleration of the controller.
//          */
//         PIDController(
//             double maxAccel,
//             double maxDecel)
//             : maxAccel(maxAccel),
//               maxDecel(maxDecel)
//         {
//         }

//         PIDController(RampParams params)
//             : maxAccel(params.maxAccel),
//               maxDecel(params.maxDecel)
//         {
//         }

//         /**
//          * Resets the ramp controller to its initial state.
//          * Should be called before starting a new control loop.
//          */
//         void reset()
//         {
//             lastError = 0;
//             lastVelocity = 0;
//             lastUpdateTimestamp = pros::millis();
//         }

//         /**
//          * Updates the ramp controller with a new target value.
//          * Should be called every control loop iteration.
//          * @param targetValue The current target value.
//          * @return The current output value of the ramp controller.
//          */
//         double update(double targetValue)
//         {
//             // Get Delta Time
//             double dt = pros::millis() - lastUpdateTimestamp;
//             lastUpdateTimestamp = pros::millis();

//             // Calculate Error
//             double error = targetValue - lastTarget;
//             double accel = (error - lastError) / dt;

//             // Calculate Velocity
//             double velocity = accel * dt;

//             // Clamp Acceleration
//             if (accel > maxAccel)
//                 accel = maxAccel;
//             else if (accel < -maxDecel)
//                 accel = -maxDecel;

//             // Calculate New Value
//             lastTarget += velocity;

//             // Return Value
//             return getValue();
//         }

//         /**
//          * Gets the current output value of the ramp controller.
//          * @return The current output value of the ramp controller.
//          */
//         double getValue()
//         {
//             return lastTarget;
//         }

//     private:
//         // Last Values
//         double lastTarget = 0;
//         double lastUpdateTimestamp = 0;

//         // PID Variables
//         double maxAccel;
//         double maxDecel;
//     };

// }