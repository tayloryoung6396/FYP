#ifndef MODULE_ONE_AXIS_HPP
#define MODULE_ONE_AXIS_HPP

#include <vector>
#include "HardwareIO/Muscle/Muscle.hpp"
#include "MPC/AdaptiveMPC/AdaptiveMPC.hpp"

namespace module {
namespace HardwareIO {
    namespace joint {

        struct axis_model_t {
            float mass;
            float radius;
            float P_a;
            float P_t;
            std::pair<float, float> limits;
            module::HardwareIO::muscle_properties_t muscle1;
            module::HardwareIO::muscle_properties_t muscle2;
        };

        class OneAxis {
        public:
            OneAxis(std::vector<module::HardwareIO::muscle_t>& muscle,
                    float mass,
                    float radius,
                    module::MPC::AdaptiveMPC::AdaptiveMPC mpc,
                    module::MPC::AdaptiveMPC::Optimizer optimizer);

            void Compute(float angle);
            void UpdateVelocity();

        private:
            // float radius;
            module::HardwareIO::Muscle muscle1;
            module::HardwareIO::Muscle muscle2;
            module::MPC::AdaptiveMPC::AdaptiveMPC mpc;
            axis_model_t axis_model;
        };

    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module

#endif  // MODULE_ONE_AXIS_HPP