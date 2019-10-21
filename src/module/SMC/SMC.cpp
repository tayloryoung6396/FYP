#include "SMC.hpp"

namespace module {
namespace SMC {

    SMC smc = SMC(1, 1, 0.05, 1, 0.01);

    SMC::SMC(float zeta, float omega, float T_s, float e_lim, float epsilon)
        : zeta(zeta), omega(omega), T_s(T_s), e_lim(e_lim), epsilon(epsilon) {
        for (int i = 1; i <= 10; i++) {
            e.push_back(0);
        }
        for (int i = 1; i <= 10; i++) {
            e_v.push_back(0);
        }
    }

    float SMC::lim(float intergral, float e_lim) {
        if (intergral > e_lim) {
            intergral = e_lim;
        }
        return (intergral);
    }

}  // namespace SMC
}  // namespace module