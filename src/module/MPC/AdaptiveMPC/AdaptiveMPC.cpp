#include "AdaptiveMPC.hpp"
#include "MPC/Model/Model.hpp"
#include "Optimizer.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {

        AdaptiveMPC::AdaptiveMPC(Optimizer optimizer) : optimizer(optimizer) {}

        std::pair<bool, bool> AdaptiveMPC::Compute(std::vector<double>& states, double theta) {

            Model::dynamic_model m;

            // Convert the input setpoint to the y length setpoint
            double setpoint = m.radius * theta;

            return (optimizer.FirstLayer(m, states, setpoint));
        }
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module