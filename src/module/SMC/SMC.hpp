#ifndef MODULE_SMC_HPP
#define MODULE_SMC_HPP

#include <vector>

namespace module {
namespace SMC {
    class SMC {
    public:
        SMC(float zeta, float omega, float T_s, float e_lim, float epsilon);

        template <typename T>
        std::pair<bool, bool> Compute(const T& model, std::vector<float>& states, float theta) {

            // e = y - y_d
            error = states[0] - (model.radius * theta);

            e.erase(e.begin());
            e.push_back(error);

            error_v = CalculateVelocity();
            error_a = CalculateAcceleration();

            // Calculate out intergral action
            integral += error * T_s;

            float smc_error = (error_a / omega * omega * omega) + (3 * zeta * error_v / omega * omega)
                              + (3 * zeta * zeta * error / omega) + lim(integral, e_lim);


            if (smc_error > epsilon) {
                return std::make_pair(true, false);
            }
            else if (smc_error < -epsilon) {
                return std::make_pair(false, true);
            }

            return std::make_pair(false, false);
        }

        float lim(float intergral, float e_lim);

        float CalculateVelocity() {
            float velocity = 0;
            int n          = e.size();
            if (n % 2 == 0) {
                // We're even
                velocity = e[0] + e[n - 1];
                for (int i = 1; i < n - 1; i += 2) {
                    velocity += 4 * e[i];
                    velocity += 2 * e[i + 1];
                }
            }
            else {
                // We're odd
                velocity = e[0] + e[n - 1];
                for (int i = 1; i < n - 1; i += 2) {
                    velocity += 4 * e[i];
                    velocity += 2 * e[i + 1];
                }
            }

            velocity = T_s * velocity / 3;

            e_v.erase(e_v.begin());
            e_v.push_back(velocity);
            return (velocity);
        }

        float CalculateAcceleration() {
            float acceleration = 0;
            int n              = e_v.size();
            if (n % 2 == 0) {
                // We're even
                acceleration = e_v[0] + e_v[n - 1];
                for (int i = 1; i < n - 1; i += 2) {
                    acceleration += 4 * e_v[i];
                    acceleration += 2 * e_v[i + 1];
                }
            }
            else {
                // We're odd
                acceleration = e_v[0] + e_v[n - 1];
                for (int i = 1; i < n - 1; i += 2) {
                    acceleration += 4 * e_v[i];
                    acceleration += 2 * e_v[i + 1];
                }
            }
            acceleration = T_s * acceleration / 3;
            return (acceleration);
        }

    private:
        float zeta;
        float omega;
        float integral;
        float T_s;
        float e_lim;
        float epsilon;
        float error;
        float error_v;
        float error_a;
        std::vector<float> e;
        std::vector<float> e_v;
    };

    extern SMC smc;
}  // namespace SMC
}  // namespace module

#endif  // MODULE_SMC_HPP