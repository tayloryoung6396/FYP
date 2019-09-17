#ifndef MODULE_PID_HPP
#define MODULE_PID_HPP

namespace shared {
namespace utility {
    class PID {
    public:
        PID(float dt, float max, float min, float Kp, float Kd, float Ki);

        float Compute(float set_point, float pv);

    private:
        //     float proportional;
        //     float intergral;
        //     float differential;
        float dt;
        float max;
        float min;
        float Kp;
        float Kd;
        float Ki;
        float pre_error;
        float integral;
    };

    extern PID pid1;
    extern PID pid2;
}  // namespace utility
}  // namespace shared

#endif  // MODULE_PID_HPP