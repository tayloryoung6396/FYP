#include "main.hpp"
#include <iostream>

#include "module/HardwareIO/Joint/OneAxis/OneAxis.hpp"
#include "module/HardwareIO/Joint/TwoAxis/TwoAxis.hpp"
#include "module/HardwareIO/Muscle/Muscle.hpp"
#include "module/HardwareIO/Valve/Valve.hpp"
#include "module/Sensors/LinearPotentiometer/LinearPotentiometer.hpp"
#include "module/Sensors/PressureSensor/PressureSensor.hpp"
#include "shared/utility/PID/PID.hpp"

int main() {
    std::cout << "Welcome to PNEUbot" << std::endl;

    module::HardwareIO::Muscle test_muscle1(module::HardwareIO::valve1,
                                            module::Sensors::pressuresensor1,
                                            module::Sensors::linearpot1,
                                            shared::utility::pid1);
    module::HardwareIO::Muscle test_muscle2(module::HardwareIO::valve2,
                                            module::Sensors::pressuresensor2,
                                            module::Sensors::linearpot2,
                                            shared::utility::pid2);

    // module::HardwareIO::joint::OneAxis Knee(std::vector<muscle_t> muscle, 0.094);

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