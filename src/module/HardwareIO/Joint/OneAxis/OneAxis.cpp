#include "OneAxis.hpp"
#include <iostream>
#include "HardwareIO/Muscle/Muscle.hpp"

namespace module {
namespace HardwareIO {
    namespace joint {

        OneAxis::OneAxis(std::vector<module::HardwareIO::muscle_t>& muscle, double radius)
            : muscle1(muscle[0]), muscle2(muscle[1]), radius(radius) {}

        void OneAxis::Compute(double angle) {
            // Set the muscles related to the joint to the set position
            // muscle1.SetPosition(angle);
        }

        // Some sort of initial position, 0
        // Some sort of initial length for all of the muscles

        // Input position

        // double L_1 = 2 * muscle_1.init_len + muscle_2.init_len - L_2;
        // double L_2 = (2 * M_PI() * OneAxis::r_1 / 360);

    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module