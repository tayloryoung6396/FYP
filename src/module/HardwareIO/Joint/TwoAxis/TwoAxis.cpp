#include "TwoAxis.hpp"
#include <iostream>

namespace module {
namespace HardwareIO {
    namespace joint {

        TwoAxis::TwoAxis() {}

        double L_1 = 2 * l_01 + l_02 + l_04 - L_2 - L_4;
        double L_2 = (2 * M_PI() * r_1 * / 360) + l_02 + l_03 - L_3;
        double L_3 = (2 * M_PI() * r_1 * / 360) + (2 * M_PI() * r_2 * / 360);
        double L_4 = (2 * M_PI() * r_2 * / 360) + l_04 + l_03 - L_3;

    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module