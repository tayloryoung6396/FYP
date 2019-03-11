#ifndef MODULE_TWO_AXIS_HPP
#define MODULE_TWO_AXIS_HPP

namespace module {
namespace HardwareIO {
    namespace joint {
        class TwoAxis {
        public:
            TwoAxis();

        private:
            double r_1;
            double r_2;

            double l_01;
            double l_02;
            double l_03;
            double l_04;
        };
    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module
#endif  // MODULE_TWO_AXIS_HPP