#ifndef MODULE_ADAPTIVE_MPC_HPP
#define MODULE_ADAPTIVE_MPC_HPP

#include <vector>

namespace module {
namespace MPC {
    namespace AdaptiveMPC {
        class AdaptiveMPC {
        public:
            AdaptiveMPC();

            std::pair<bool, bool> Compute(std::vector<double>& states, double theta);
        };
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module

#endif  // MODULE_ADAPTIVE_MPC_HPP
