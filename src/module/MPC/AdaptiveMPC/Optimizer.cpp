// For each controller mode, recursively call the optimizer control horizon number of times
// Once a leg is complete, append to the cost list the cost value and the root action of the leg

#include "Optimizer.hpp"
#include "MPC/Model/Model.hpp"
#include "utility/io/uart.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {
        Optimizer optimizer1 = Optimizer(40, 0, 0);

        Optimizer::Optimizer(int ch_max, float state_weight, float input_weight)
            : ch_max(ch_max), state_weight(state_weight), input_weight(input_weight) {}
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module