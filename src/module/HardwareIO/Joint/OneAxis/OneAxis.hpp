#ifndef MODULE_ONE_AXIS_HPP
#define MODULE_ONE_AXIS_HPP

namespace module {
namespace HardwareIO {
    namespace joint {
        class OneAxis {
        public:
            OneAxis(double r_1);

        private:
            double r_1;
        };
    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module

#endif  // MODULE_ONE_AXIS_HPP