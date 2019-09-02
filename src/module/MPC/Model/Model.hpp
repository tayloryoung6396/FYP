#ifndef MODULE_OPTIMIZER_HPP
#define MODULE_OPTIMIZER_HPP

#include <vector>

namespace module {
namespace MPC {
    namespace Model {

        struct dynamic_model {
            void Linearize();
            // const double a_xx1[4][4];
            // const double a_xx2[4][4];
        };

    }  // namespace Model
}  // namespace MPC
}  // namespace module

#endif  // MODULE_OPTIMIZER_HPP
