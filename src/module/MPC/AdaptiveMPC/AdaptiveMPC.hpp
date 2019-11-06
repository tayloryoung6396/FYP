#ifndef MODULE_ADAPTIVE_MPC_HPP
#define MODULE_ADAPTIVE_MPC_HPP

#define _USE_MATH_DEFINES
#define M_PI 3.14159265358979323846

#include <math.h>
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

                std::vector<float> setpoint = {model.radius * theta, 0, 0, 0};

                setpoint = {model.radius * theta, 0, 413685, 413685};
                // utility::io::debug.out(
                //     "Setpoint %f\t, %f\t, %f\t, %f\n", setpoint[0], setpoint[1], setpoint[2], setpoint[3]);
                // utility::io::debug.out("Running MPC Compute\n");

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
