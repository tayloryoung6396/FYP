#ifndef MODULE_ADAPTIVE_MPC_HPP
#define MODULE_ADAPTIVE_MPC_HPP

#include <vector>
#include "Optimizer.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {
        class AdaptiveMPC {
        public:
            AdaptiveMPC(module::MPC::AdaptiveMPC::Optimizer optimizer);

            template <typename T>
            std::pair<bool, bool> Compute(const T& model, std::vector<float>& states, float theta) {

                // Convert the input setpoint to the y length setpoint
                theta = 0;  // TODO Remove

                std::vector<float> setpoint = {model.radius * theta, 0, 0, 0};

                setpoint = {0, 0, 0, 0};
                utility::io::debug.out("Running MPC Compute\n");

                return (optimizer.FirstLayer(model, states, setpoint));
            }

        private:
            module::MPC::AdaptiveMPC::Optimizer optimizer;
        };

        extern AdaptiveMPC mpc;
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module

#endif  // MODULE_ADAPTIVE_MPC_HPP
