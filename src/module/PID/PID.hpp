#ifndef MODULE_PID_HPP
#define MODULE_PID_HPP

#include <vector>

namespace module {
namespace PID {
    class PID {
    public:
        PID(float dt, float max, float min, float Kp, float Kd, float Ki);

        std::pair<bool, bool> Compute(float set_point, float pv);

    private:
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
}  // namespace PID
}  // namespace module

#endif  // MODULE_PID_HPP