#include "Muscle.hpp"
#include <stdint.h>
#include <iostream>
#include <vector>
#include "utility/io/gpio.hpp"
#include "utility/io/uart.hpp"
#include "utility/math/median.hpp"

namespace module {
namespace HardwareIO {

    Muscle::Muscle(module::HardwareIO::muscle_t muscle)
        : valve(muscle.valve)
        , pressure_sensor(muscle.pressure_sensor)
        , linear_pot(muscle.linear_pot)
        , properties(muscle.properties) {
        // Populate our previous position vector to all 0
        for (int i = 0; i < 10; i++) {
            prev_position.push_back(0);
        }
        utility::io::debug.out("Muscle Initialisation\n");
    }


    float Muscle::GetPosition() { return linear_pot.GetPosition(); }

    void Muscle::UpdateVelocity() {
        prev_position.erase(prev_position.begin());
        prev_position.push_back(GetPosition());
    }

    float Muscle::GetVelocity() {
        // https://en.wikipedia.org/wiki/Median_filter
        // To demonstrate, using a window size of three with one entry immediately preceding and following each entry, a
        // median filter will be applied to the following simple 1D signal:

        // TODO Fix this

        const int w = 1;  // This is the window size for the medians either side of the center value
        std::vector<float> velocity;
        for (int i = 0; i < 10; i++) {
            std::vector<float> position;
            for (int j = -w; j < w; j++) {
                if (i + j < 0) {
                    j++;
                }
                else if (i + j > prev_position.size()) {
                    break;
                }
                position.push_back(prev_position[i + j]);
            }
            velocity.push_back(utility::math::median(position));
            position.clear();
        }

        // Now the discrete difference needs to be calculated and returned

        float Sampling_time3 = 0.05;  // 0.01 T_s

        float vel_avg = 0;
        for (int v = 1; v < velocity.size(); v++) {
            vel_avg += (velocity[v] - velocity[v - 1]) / Sampling_time3;
        }

        // return (vel_avg / (velocity.size() - 1));
        return (0);
    }

    float Muscle::GetPressure() { return pressure_sensor.GetPressure(); }

    void Muscle::SetValveState(bool state) { valve = state; }

    bool Muscle::GetValveState() { return valve; }
}  // namespace HardwareIO
}  // namespace module