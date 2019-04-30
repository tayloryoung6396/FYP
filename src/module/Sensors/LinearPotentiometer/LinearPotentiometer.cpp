#include "LinearPotentiometer.hpp"
#include <iostream>

namespace module {
namespace Sensors {

    LinearPot linearpot1 = LinearPot(1);
    // LinearPot linearpot2  = LinearPot(2);
    // LinearPot linearpot3  = LinearPot(3);
    // LinearPot linearpot4  = LinearPot(4);
    // LinearPot linearpot5  = LinearPot(5);
    // LinearPot linearpot6  = LinearPot(6);
    // LinearPot linearpot7  = LinearPot(7);
    // LinearPot linearpot8  = LinearPot(8);
    // LinearPot linearpot9  = LinearPot(9);
    // LinearPot linearpot10 = LinearPot(10);
    // LinearPot linearpot11 = LinearPot(11);
    // LinearPot linearpot12 = LinearPot(12);

    LinearPot::LinearPot(uint32_t gpio) {}

    double LinearPot::GetPosition() {
        // Read gpio
        position = 5;
        std::cout << "Position at " << position << std::endl;
        position = ConvertPotentiometer(shared::utility::adc_io.GetSensors(gpio));
        return position;
    }

    double LinearPot::ConvertPotentiometer(double Potentiometer) {

        Potentiometer = Potentiometer;  // TODO Scale by the adc ammount and then the length scale to m
        return Potentiometer;
    }
}  // namespace Sensors
}  // namespace module