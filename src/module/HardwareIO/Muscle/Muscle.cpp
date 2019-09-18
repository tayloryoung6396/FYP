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
        , properties(muscle.properties) {

        utility::io::debug.out("Initialising Muscle\n");

        utility::io::debug.out("\tnom_length %lf\n", properties.nom_length);
        utility::io::debug.out("\tcontraction_percent %lf\n", properties.contraction_percent);
        utility::io::debug.out("\tdiameter %lf\n", properties.diameter);
    }

    void Muscle::SetValveState(bool state) { valve = state; }

    float Muscle::GetPosition() { return linear_pot.GetPosition(); }

    float Muscle::GetPressure() { return pressure_sensor.GetPressure(); }

    bool Muscle::GetValveState() { return valve; }
}  // namespace HardwareIO
}  // namespace module