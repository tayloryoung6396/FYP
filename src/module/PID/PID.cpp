#include "PID.hpp"

namespace shared {
namespace utility {

    PID pid1 = PID(0.1, 100, -100, 0.1, 0.01, 0.5);
    PID pid2 = PID(0.1, 100, -100, 0.1, 0.01, 0.5);

    PID::PID(float dt, float max, float min, float Kp, float Kd, float Ki)
        : dt(dt), max(max), min(min), Kp(Kp), Kd(Kd), Ki(Ki), pre_error(0), integral(0) {}

    float PID::Compute(float setpoint, float pv) {

        // Calculate error
        float error = setpoint - pv;

        // Proportional term
        float Pout = Kp * error;

        // Integral term
        integral += error * dt;
        float Iout = Ki * integral;

        // Derivative term
        float derivative = (error - pre_error) / dt;
        float Dout       = Kd * derivative;

        // Calculate total output
        float output = Pout + Iout + Dout;

        // Restrict to max/min
        if (output > max) {
            output = max;
        }
        else if (output < min) {
            output = min;
        }

        // Save error to previous error
        pre_error = error;

        return output;
    }

}  // namespace utility
}  // namespace shared