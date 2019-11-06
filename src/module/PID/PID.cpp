#include "PID.hpp"
#include "utility/io/uart.hpp"

namespace module {
namespace PID {

    PID pid = PID(0.05, 100, 0, 0);
    // PID pid = PID(0.5, 0.6 * 100, 1.2 * 100 / 0.05, 3 * 100 * 0.05 / 40.0);

    PID::PID(float dt, float Kp, float Kd, float Ki) : dt(dt), Kp(Kp), Kd(Kd), Ki(Ki), pre_error(0), integral(0) {}

    std::pair<bool, bool> PID::Compute(float setpoint, float pv) {

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

        // Save error to previous error
        pre_error = error;

        // Restrict to max/min
        if (output < -0.01) {
            // utility::io::debug.out("true, false, error %lf", error);
            utility::io::debug.out("%lf, ", output);
            return std::make_pair(true, false);
        }
        else if (output > 0.01) {
            // utility::io::debug.out("false, true, error %lf", error);
            utility::io::debug.out("%lf, ", output);
            return std::make_pair(false, true);
        }
        // utility::io::debug.out("false, false, error %lf", error);
        utility::io::debug.out("%lf, ", output);
        return std::make_pair(false, false);
    }

}  // namespace PID
}  // namespace module