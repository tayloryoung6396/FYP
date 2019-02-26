#ifndef MODULE_PID_HPP
#define MODULE_PID_HPP

namespace shared {
namespace utility {
    class PID {
    public:
        PID();

        void Compute(double set_point);

    private:
        double proportional;
        double intergral;
        double differential;
    };

    extern PID pid1;
    extern PID pid2;
}  // namespace utility
}  // namespace shared

#endif  // MODULE_PID_HPP