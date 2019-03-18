#include "StaticModel.hpp"

namespace shared {
namespace utility {

    StaticModel::StaticModel() {}

    double StaticModel::CalculateForce() {
        // Math

        double len_f = len_0 / std::cos(theta_0);

        double n = (len_0 * std::tan(theta_0)) / (M_PI() * D_0);

        double delta_D = sqrt(len_f ^ 2 - length ^ 2) / (n * M_PI());

        double delta_V = ((length * len_f ^ 2) / (4 * M_PI() * n ^ 2)) - ((length ^ 3) / (4 * M_PI() * n ^ 2));

        double E_ru = c_3 * length ^ 3 + c_2 * length ^ 2 + c_1 * length ^ 1 + c_0;

        double sig_pe = E_ru * (delta_D - D_0) / D_0;

        double sig_l = E_ru * (length - len_0) / len_0;

        double F_pe = sig_pe * H_0 * length * M_PI();

        double F_l = sig_l * H_0 * M_PI() * delta_D;

        double F_m = -pressure * delta_V + F_pe * delta_D - F_l;
        // double W_pma = W_vae + W_el;
    }
}  // namespace utility
}  // namespace shared