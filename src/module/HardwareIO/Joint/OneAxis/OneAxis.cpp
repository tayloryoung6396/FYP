#include "OneAxis.hpp"
#include <iostream>
#include "utility/math/saturation.hpp"

namespace module {
namespace HardwareIO {
    namespace joint {

        OneAxis::OneAxis(std::vector<module::HardwareIO::muscle_t>& muscle,
                         float mass,
                         float radius,
                         module::MPC::AdaptiveMPC::AdaptiveMPC mpc,
                         module::MPC::AdaptiveMPC::Optimizer optimizer)
            : muscle1(muscle[0]), muscle2(muscle[1]), mpc(optimizer) {
            axis_model.mass    = mass;
            axis_model.radius  = radius;
            axis_model.P_a     = 0;
            axis_model.muscle1 = muscle[0].properties;
            axis_model.muscle2 = muscle[1].properties;
            axis_model.limits  = std::make_pair(-M_PI / 2.0, M_PI / 2.0);
            utility::io::debug.out("OneAxis Initialisation\n");
        }

        // TODO I think this is how the system needs to work
        // Joints know about the muscles, a joint contains all of the muscles it needs to perform. Each muscle has
        // knowledge of its respective sensors, pressure, position. A muscle is responsible for reading it's sensors.
        // The joint compute function then calls the appropriate MPC function and arranges the inputs how the MPC
        // expects. Something like this

        void OneAxis::Compute(float theta) {
            // float muscle1GetPosition = muscle1.GetPosition();
            // float muscle1GetVelocity = muscle1.GetVelocity();
            // float muscle1GetPressure = muscle1.GetPressure();
            // float muscle2GetPosition = muscle2.GetPosition();
            // float muscle2GetVelocity = muscle2.GetVelocity();
            // float muscle2GetPressure = muscle2.GetPressure();

            // utility::io::debug.out(
            //     "m1 %.2f | %.2f | %.2f | ", muscle1GetPosition, muscle1GetVelocity, muscle1GetPressure);
            // utility::io::debug.out(
            //     "m2 %.2f | %.2f | %.2f\n", muscle2GetPosition, muscle2GetVelocity, muscle2GetPressure);

            // Calculate the position derivative for velocity
            // Append states to a state vector in the order required
            // TODO Measure P_t
            axis_model.P_t = 413685;  // 60 PSI

            std::vector<float> states = {0.05, 0, 413685, 413685};
            //{muscle1.GetPosition(), muscle1.GetVelocity(), muscle1.GetPressure(), muscle2.GetPressure()};

            // Saturate the input to something sensible
            theta = utility::math::sat(theta, axis_model.limits);

            // // Call the MPC compute
            std::pair<bool, bool> valve_state = mpc.Compute(axis_model, states, theta);

            muscle1.SetValveState(valve_state.first);
            muscle2.SetValveState(valve_state.second);
        }

        void OneAxis::UpdateVelocity() {
            muscle1.UpdateVelocity();
            // muscle2.UpdateVelocity();
        }

    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module

// Axis compute should pass through the setpoint, the state vector that i care about and a model.
