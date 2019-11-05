// For each controller mode, recursively call the optimizer control horizon number of times
// Once a leg is complete, append to the cost list the cost value and the root action of the leg

#include "Optimizer.hpp"
#include "utility/io/uart.hpp"

namespace module {
namespace Simulator {

    Plant plant = Plant();

    Plant::Plant(std::vector<module::HardwareIO::muscle_t>& muscle, float mass, float radius)
        : muscle1(muscle[0]), muscle2(muscle[1]) {
        axis_model.mass    = mass;
        axis_model.radius  = radius;
        axis_model.P_a     = 101325;  // 1 atm
        axis_model.muscle1 = muscle[0].properties;
        axis_model.muscle2 = muscle[1].properties;
        axis_model.limits  = std::make_pair(-M_PI / 2.0, M_PI / 2.0);
        utility::io::debug.out("Plant Initialisation\n");
    }

    void Compute() {
        axis_model.P_t = 413685;  // 60 PSI

        std::vector<float> states = {
            muscle1.GetPosition(), muscle1.GetVelocity(), muscle1.GetPressure(), muscle2.GetPressure()};

        std::vector<float>& output_states;
        ProcessModel(m, states, valve_states, output_states);
    }

}  // namespace Simulator
}  // namespace module
