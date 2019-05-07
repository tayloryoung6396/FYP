#include "Muscle.hpp"
#include <stdint.h>
#include <iostream>
#include "gpio.h"
#include "utility/io/uart.hpp"

namespace module {
namespace HardwareIO {

    Muscle::Muscle(module::HardwareIO::muscle_t muscle)
        : valve(muscle.valve)
        , pressure_sensor(muscle.pressure_sensor)
        , linear_pot(muscle.linear_pot)
        , pid(muscle.pid)
        , length(muscle.length) {}

    void Muscle::SetPosition(double set_point) {
        utility::io::debug.out("Set Point %lf\n", set_point);

        // Get our sensor data from raw sensors
        double pressure = pressure_sensor.GetPressure();
        double position = linear_pot.GetPosition();

        // Decide how we want to act
        double value   = position;
        double control = pid.Compute(set_point, value);

        utility::io::debug.out("Control %lf\n", control);

        // From the control point decide how to act on the valve to reach a required state
        if (control > 0) {
            valve = true;
            // TODO This should also set the time horizon variables
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PinState(true));
            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PinState(false));
        }
        else if (control < 0) {
            valve = false;
            // TODO This should also set the time horizon variables
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PinState(false));
            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PinState(true));
        }
        else {
            HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PinState(true));
            HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PinState(true));
        }
    }

    double Muscle::GetPosition() { return linear_pot.GetPosition(); }

    double Muscle::GetPressure() { return pressure_sensor.GetPressure(); }

    bool Muscle::GetValveState() { return valve; }
}  // namespace HardwareIO
}  // namespace module