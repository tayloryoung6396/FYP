#include "AdaptiveMPC.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {

        AdaptiveMPC::AdaptiveMPC(Optimizer optimizer) : optimizer(optimizer) {
            utility::io::debug.out("Adaptive MPC Initialisation\n");
        }

    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module