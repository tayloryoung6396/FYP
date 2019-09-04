#ifndef MODULE_PID_HPP
#define MODULE_PID_HPP

namespace shared {
namespace utility {
    class PID {
    public:
        PID(double dt, double max, double min, double Kp, double Kd, double Ki);

        double Compute(double set_point, double pv);

    private:
        //     double proportional;
        //     double intergral;
        //     double differential;
        double dt;
        double max;
        double min;
        double Kp;
        double Kd;
        double Ki;
        double pre_error;
        double integral;
    };

    extern PID pid1;
    extern PID pid2;
}  // namespace utility
}  // namespace shared

#endif  // MODULE_PID_HPP