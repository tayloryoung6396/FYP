#ifndef MODULE_ADAPTIVE_MPC_HPP
#define MODULE_ADAPTIVE_MPC_HPP

#include <vector>
#include "MPC/Model/Model.hpp"
#include "Optimizer.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {
        class AdaptiveMPC {
        public:
            AdaptiveMPC(module::MPC::AdaptiveMPC::Optimizer optimizer);

            std::pair<bool, bool> Compute(std::vector<float>& states, float theta);

        private:
            module::MPC::AdaptiveMPC::Optimizer optimizer;
        };

        extern AdaptiveMPC mpc;
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module

#endif  // MODULE_ADAPTIVE_MPC_HPP
