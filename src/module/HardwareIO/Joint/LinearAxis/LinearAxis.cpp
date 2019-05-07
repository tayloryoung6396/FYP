#include "LinearAxis.hpp"
#include <iostream>
#include "HardwareIO/Muscle/Muscle.hpp"

namespace module {
namespace HardwareIO {
    namespace joint {

        LinearAxis::LinearAxis(std::vector<module::HardwareIO::muscle_t>& muscle) : muscle1(muscle[0]) {}

        void LinearAxis::Compute(double position) {
            // Set each muscle related to the joint to the set position
            muscle1.SetPosition(position);
        }

    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module