#include "OneAxis.hpp"
#include <iostream>
#include "utility/math/saturation.hpp"

namespace module {
namespace HardwareIO {
    namespace joint {


        // TODO I think this is how the system needs to work
        // Joints know about the muscles, a joint contains all of the muscles it needs to perform. Each muscle has
        // knowledge of its respective sensors, pressure, position. A muscle is responsible for reading it's sensors.
        // The joint compute function then calls the appropriate MPC function and arranges the inputs how the MPC
        // expects. Something like this


    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module

// Axis compute should pass through the setpoint, the state vector that i care about and a model.
