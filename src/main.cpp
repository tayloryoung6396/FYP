#include "main.hpp"
#include <iostream>
#include "module/Joint/OneAxis/OneAxis.hpp"
#include "module/Joint/TwoAxis/TwoAxis.hpp"
#include "module/LinearPotentiometer/LinearPotentiometer.hpp"
#include "module/Muscle/Muscle.hpp"
#include "module/PID/PID.hpp"
#include "module/PressureSensor/PressureSensor.hpp"
#include "module/Valve/Valve.hpp"

int main() {
    std::cout << "Welcome to PNEUbot" << std::endl;

    module::Muscle test_muscle1(module::valve1, module::Sensors::pressuresensor1, module::linearpot1, module::pid1);
    module::Muscle test_muscle2(module::valve2, module::Sensors::pressuresensor2, module::linearpot2, module::pid2);


    while (1) {
        // Read in requested set position

        test_muscle1.SetPosition(100);
        test_muscle1.GetPosition();
        // Read sensors{
        // Read current position
        // Read current pressure
        // }
        // Input into moving control filter
        // Output PWM or time
        // Call valve timing function
    }
}