#include "PID.hpp"

namespace shared {
namespace utility {

    PID pid1 = PID();
    PID pid2 = PID();

    PID::PID() {}  // PressureSensor& pressure_sensor, LinearPot& linear_pot)
    //: pressure_sensor(pressure_sensor), linear_pot(linear_pot) {}

    void PID::Compute() {}

    // // Get the pin state from the MCU for the pin and return it
    // PID::operator bool() { return bool(GPIO_PinState(gpio_pin)); }

    // bool PID::operator=(const bool& value) {
    //     GPIO_WritePin(gpio_pin, GPIO_PinState(value));
    //     return value;
    // }
    // bool PID::operator!() {
    //     GPIO_TogglePin(gpio_pin);
    //     return this->operator bool();
    // }
}  // namespace utility
}  // namespace shared


// #ifndef _PID_SOURCE_
// #define _PID_SOURCE_

// #include <cmath>
// #include <iostream>
// #include "pid.h"

// using namespace std;

// class PIDImpl {
// public:
//     PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki);
//     ~PIDImpl();
//     double calculate(double setpoint, double pv);

// private:
//     double _dt;
//     double _max;
//     double _min;
//     double _Kp;
//     double _Kd;
//     double _Ki;
//     double _pre_error;
//     double _integral;
// };


// PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki) {
//     pimpl = new PIDImpl(dt, max, min, Kp, Kd, Ki);
// }
// double PID::calculate(double setpoint, double pv) { return pimpl->calculate(setpoint, pv); }
// PID::~PID() { delete pimpl; }


// /**
//  * Implementation
//  */
// PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki)
//     : _dt(dt), _max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0), _integral(0) {}

// double PIDImpl::calculate(double setpoint, double pv) {

//     // Calculate error
//     double error = setpoint - pv;

//     // Proportional term
//     double Pout = _Kp * error;

//     // Integral term
//     _integral += error * _dt;
//     double Iout = _Ki * _integral;

//     // Derivative term
//     double derivative = (error - _pre_error) / _dt;
//     double Dout       = _Kd * derivative;

//     // Calculate total output
//     double output = Pout + Iout + Dout;

//     // Restrict to max/min
//     if (output > _max)
//         output = _max;
//     else if (output < _min)
//         output = _min;

//     // Save error to previous error
//     _pre_error = error;

//     return output;
// }

// PIDImpl::~PIDImpl() {}

// #endif


// #include "pid.h"
// #include <stdio.h>

// int main() {

//     PID pid = PID(0.1, 100, -100, 0.1, 0.01, 0.5);

//     double val = 20;
//     for (int i = 0; i < 100; i++) {
//         double inc = pid.calculate(0, val);
//         printf("val:% 7.3f inc:% 7.3f\n", val, inc);
//         val += inc;
//     }

//     return 0;
// }