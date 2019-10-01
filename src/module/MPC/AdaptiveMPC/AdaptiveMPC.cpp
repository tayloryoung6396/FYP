#include "AdaptiveMPC.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {

        AdaptiveMPC mpc = AdaptiveMPC(optimizer1);

        AdaptiveMPC::AdaptiveMPC(Optimizer optimizer) : optimizer(optimizer) {}

    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module