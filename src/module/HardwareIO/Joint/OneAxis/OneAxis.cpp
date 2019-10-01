#include "OneAxis.hpp"
#include <iostream>

namespace module {
namespace HardwareIO {
    namespace joint {

        OneAxis::OneAxis(std::vector<module::HardwareIO::muscle_t>& muscle,
                         float mass,
                         float radius,
                         module::MPC::AdaptiveMPC::AdaptiveMPC mpc)
            : muscle1(muscle[0]), muscle2(muscle[1]), mpc(mpc) {
            axis_model.mass    = mass;
            axis_model.radius  = radius;
            axis_model.P_a     = 0;
            axis_model.muscle1 = muscle[0].properties;
            axis_model.muscle2 = muscle[1].properties;
        }

        // TODO I think this is how the system needs to work
        // Joints know about the muscles, a joint contains all of the muscles it needs to perform. Each muscle has
        // knowledge of its respective sensors, pressure, position. A muscle is responsible for reading it's sensors.
        // The joint compute function then calls the appropriate MPC function and arranges the inputs how the MPC
        // expects. Something like this

        void OneAxis::Compute(float theta) {
            utility::io::debug.out("M1Po %.2f\t", muscle1.GetPosition());
            utility::io::debug.out("Pr %.2f\t", muscle1.GetPressure());
            utility::io::debug.out("M2Po %.2f\t", muscle2.GetPosition());
            utility::io::debug.out("Pr %.2f\n", muscle2.GetPressure());

            // TODO Saturate the input to something sensible
            // TODO Calculate the position derivative for velocity
            // TODO append these to a state vector in the order required
            // TODO NOTE None of this will work until i figure out the relevant states, the VolumeAIr state and
            // outputstate/cost might need to update
            // TODO Measure P_t
            float P_t                 = 60;
            std::vector<float> states = {muscle1.GetPosition(),
                                         muscle1.GetVelocity(),
                                         muscle1.GetPressure(),
                                         muscle2.GetPosition(),
                                         muscle1.GetVelocity(),
                                         muscle2.GetPressure(),
                                         P_t};

            // // Call the MPC compute
            std::pair<bool, bool> valve_state = mpc.Compute(axis_model, states, theta);

            muscle1.SetValveState(valve_state.first);
            muscle2.SetValveState(valve_state.second);
        }

        void OneAxis::UpdateVelocity() {
            muscle1.UpdateVelocity();
            muscle2.UpdateVelocity();
        }

    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module

// Axis compute should pass through the setpoint, the state vector that i care about and a model.
