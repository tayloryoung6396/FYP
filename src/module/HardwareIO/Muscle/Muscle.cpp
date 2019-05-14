#include "Muscle.hpp"
#include <stdint.h>
#include <iostream>
#include "utility/io/gpio.hpp"
#include "utility/io/uart.hpp"

namespace module {
namespace HardwareIO {

    Muscle::Muscle(module::HardwareIO::muscle_t muscle)
        : valve(muscle.valve)
        , pressure_sensor(muscle.pressure_sensor)
        , linear_pot(muscle.linear_pot)
        , pid(muscle.pid)
        , properties(muscle.properties) {

        utility::io::debug.out("Initialising Muscle\n");

        utility::io::debug.out("\tnom_length %lf\n", properties.nom_length);
        utility::io::debug.out("\tcontraction_percent %lf\n", properties.contraction_percent);
        utility::io::debug.out("\tdiameter %lf\n", properties.diameter);
    }

    void Muscle::SetPosition(double set_point) {
        utility::io::debug.out("Set Point %lf\n", set_point);

        // Get our sensor data from raw sensors
        // double pressure = pressure_sensor.GetPressure();
        double position = linear_pot.GetPosition();

        // // Decide how we want to act
        // double value   = position;
        // double control = pid.Compute(set_point, value);

        // utility::io::debug.out("Control %lf\n", control);

        // // From the control point decide how to act on the valve to reach a required state
        // if (control > 0) {
        //     valve = true;
        //     // TODO This should also set the time horizon variables
        //     utility::io::gpio::led2 = true;
        //     utility::io::gpio::led3 = false;
        // }
        // else if (control < 0) {
        //     valve = false;
        //     // TODO This should also set the time horizon variables
        //     utility::io::gpio::led2 = false;
        //     utility::io::gpio::led3 = true;
        // }
        // else {
        //     utility::io::gpio::led2 = true;
        //     utility::io::gpio::led3 = true;
        // }
    }

    double Muscle::GetPosition() { return linear_pot.GetPosition(); }

    double Muscle::GetPressure() { return pressure_sensor.GetPressure(); }

    bool Muscle::GetValveState() { return valve; }
}  // namespace HardwareIO
}  // namespace module