#include "AdaptiveMPC.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {

        AdaptiveMPC mpc = AdaptiveMPC(optimizer1);

        AdaptiveMPC::AdaptiveMPC(Optimizer optimizer) : optimizer(optimizer) {}

        std::pair<bool, bool> AdaptiveMPC::Compute(std::vector<float>& states, float theta) {

            Model::dynamic_model m;

            // Convert the input setpoint to the y length setpoint
            float setpoint = m.radius * theta;

            return (optimizer.FirstLayer(m, states, setpoint));
        }
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module