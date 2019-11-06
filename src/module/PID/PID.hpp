#ifndef MODULE_PID_HPP
#define MODULE_PID_HPP

#include <vector>

namespace module {
namespace PID {
    class PID {
    public:
        PID(float dt, float Kp, float Kd, float Ki);

        std::pair<bool, bool> Compute(float set_point, float pv);

        template <typename T>
        std::pair<bool, bool> Compute(const T& m, std::vector<float>& states, float setpoint) {
            return (Compute(m.radius * setpoint, states[0]));
        }

    private:
        float dt;
        float Kp;
        float Kd;
        float Ki;
        float pre_error;
        float integral;
    };

    extern PID pid;
}  // namespace PID
}  // namespace module

#endif  // MODULE_PID_HPP