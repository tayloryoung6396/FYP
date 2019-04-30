#ifndef MODULE_LINEAR_AXIS_HPP
#define MODULE_LINEAR_AXIS_HPP

#include <vector>
#include "HardwareIO/Muscle/Muscle.hpp"

namespace module {
namespace HardwareIO {
    namespace joint {
        class LinearAxis {
        public:
            LinearAxis(std::vector<module::HardwareIO::muscle_t>& muscle);

            void Compute(double position);

        private:
            module::HardwareIO::Muscle muscle1;
        };
    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module

#endif  // MODULE_LINEAR_AXIS_HPP