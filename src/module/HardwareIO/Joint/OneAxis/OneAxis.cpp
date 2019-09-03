#include "OneAxis.hpp"
#include <iostream>

namespace module {
namespace HardwareIO {
    namespace joint {

        OneAxis::OneAxis(std::vector<module::HardwareIO::muscle_t>& muscle, double radius, AdaptiveMPC mpc)
            : muscle1(muscle[0]), muscle2(muscle[1]), radius(radius), mpc(mpc) {}

        // TODO I think this is how the system needs to work
        // Joints know about the muscles, a joint contains all of the muscles it needs to perform. Each muscle has
        // knowledge of its respective sensors, pressure, position. A muscle is responsible for reading it's sensors.
        // The joint compute function then calls the appropriate MPC function and arranges the inputs how the MPC
        // expects. Something like this

        void OneAxis::Compute(double theta) {
            muscle1.GetPosition();
            muscle1.GetPressure();
            muscle2.GetPosition();
            muscle2.GetPressure();

            // TODO Calculate the position derivative for velocity
            // TODO append these to a state vector in the order required
            std::vector<double> states;

            // Call the MPC compute
            std::pair<bool, bool> valve_state = mpc.Compute(states, theta);

            muscle1.SetValveState(valve_state.first);
            muscle2.SetValveState(valve_state.second);
        }

    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module