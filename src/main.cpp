#include "main.hpp"
#include <iostream>
#include "module/LinearPotentiometer/LinearPotentiometer.hpp"
#include "module/Muscle/Muscle.hpp"
#include "module/PID/PID.hpp"
#include "module/PressureSensor/PressureSensor.hpp"
#include "module/Valve/Valve.hpp"

int main() {
    std::cout << "Hello World!" << std::endl;

    module::Muscle test_muscle(module::Valve, module::pressure_sensor::p_01);

    while (1) {
        // Read in requested set position
        // Read sensors{
        // Read current position
        // Read current pressure
        // }
        // Input into moving control filter
        // Output PWM or time
        // Call valve timing function
    }
}