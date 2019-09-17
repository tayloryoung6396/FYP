#ifndef MODULE_TWO_AXIS_HPP
#define MODULE_TWO_AXIS_HPP

namespace module {
namespace HardwareIO {
    namespace joint {
        class TwoAxis {
        public:
            TwoAxis();

        private:
            float r_1;
            float r_2;

            float l_01;
            float l_02;
            float l_03;
            float l_04;
        };
    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module
#endif  // MODULE_TWO_AXIS_HPP