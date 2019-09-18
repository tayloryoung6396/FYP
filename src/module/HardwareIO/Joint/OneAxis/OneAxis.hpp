#ifndef MODULE_ONE_AXIS_HPP
#define MODULE_ONE_AXIS_HPP

#include <vector>
#include "HardwareIO/Muscle/Muscle.hpp"
#include "MPC/AdaptiveMPC/AdaptiveMPC.hpp"

namespace module {
namespace HardwareIO {
    namespace joint {
        class OneAxis {
        public:
            OneAxis(std::vector<module::HardwareIO::muscle_t>& muscle,
                    float radius,
                    module::MPC::AdaptiveMPC::AdaptiveMPC mpc);

            void Compute(float angle);

        private:
            float radius;
            module::HardwareIO::Muscle muscle1;
            module::HardwareIO::Muscle muscle2;
            module::MPC::AdaptiveMPC::AdaptiveMPC mpc;
        };

    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module

#endif  // MODULE_ONE_AXIS_HPP