#ifndef MODULE_MODEL_HPP
#define MODULE_MODEL_HPP

#include <vector>
#include "HardwareIO/Muscle/Muscle.hpp"

namespace module {
namespace MPC {
    namespace Model {

        struct mode_t {
            double A1[4][4];
            double A2[4][4];
        };

        struct dynamic_model {
            module::HardwareIO::muscle_properties_t muscle1;
            module::HardwareIO::muscle_properties_t muscle2;
            mode_t mode1;
            mode_t mode2;
            mode_t mode3;
            double radius;
        };

        // struct muscle_properties_t {
        //     double nom_length;
        //     double contraction_percent;
        //     double diameter;
        //     double a_xx[6][6];
        // };

    }  // namespace Model
}  // namespace MPC
}  // namespace module

#endif  // MODULE_MODEL_HPP
