#ifndef MODULE_ADAPTIVE_MPC_HPP
#define MODULE_ADAPTIVE_MPC_HPP

#include <vector>
#include "Optimizer.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {
        class AdaptiveMPC {
        public:
            AdaptiveMPC(Optimizer optimizer);

            std::pair<bool, bool> Compute(std::vector<double>& states, double theta);

        private:
            Optimizer& optimizer;
        };
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module

#endif  // MODULE_ADAPTIVE_MPC_HPP
