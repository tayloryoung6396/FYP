#include "OneAxis.hpp"
#include <iostream>

namespace module {
namespace HardwareIO {
    namespace joint {

        OneAxis::OneAxis(double r_1) {

            // module::HardwareIO::Muscle muscle_1(module::HardwareIO::valve1,
            //                                     module::Sensors::pressuresensor1,
            //                                     module::Sensors::linearpot1,
            //                                     shared::utility::pid1);
            // module::HardwareIO::Muscle muscle_2(module::HardwareIO::valve2,
            //                                     module::Sensors::pressuresensor2,
            //                                     module::Sensors::linearpot2,
            //                                     shared::utility::pid2);
        }

        // Some sort of initial position, 0
        // Some sort of initial length for all of the muscles

        // Input position

        // double L_1 = 2 * muscle_1.init_len + muscle_2.init_len - L_2;
        // double L_2 = (2 * M_PI() * OneAxis::r_1 / 360);

    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module