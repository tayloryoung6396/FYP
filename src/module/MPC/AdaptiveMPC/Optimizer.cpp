// For each controller mode, recursively call the optimizer control horizon number of times
// Once a leg is complete, append to the cost list the cost value and the root action of the leg

#include "Optimizer.hpp"
#include "utility/io/uart.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {

        Optimizer optimizer1 = Optimizer(3);

        Optimizer::Optimizer(int ch_max) : ch_max(ch_max) {
            utility::io::debug.out("Optimizer Initialisation\n");
            /* clang-format off */
            state_weight << 1000, 0, 0,     0, 
            				0,    1, 0,     0, 
            				0,    0, 0.001, 0,
            				0,    0, 0,     0.001;
            /* clang-format on */
        }
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module