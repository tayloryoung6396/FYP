#ifndef MODULE_OPTIMIZER_HPP
#define MODULE_OPTIMIZER_HPP

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <vector>
#include "main.h"
#include "utility/io/uart.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {
        class Optimizer {
        public:
            Optimizer(int ch_max, std::vector<float> state_weight, float input_weight);

            template <typename T>
            std::pair<std::vector<float>, float> ProcessModel(const T& model,
                                                              int mode,
                                                              std::vector<float>& states,
                                                              float setpoint,
                                                              std::vector<float>& output_states) {

                // Given a control action, select the correct model matrix, from that linearize the model about the
                // current state point. Calculate the next state vector. Call the control error function and return the
                // errors
                if (mode == 1) {
                    Linearise(model,
                              states,
                              setpoint,
                              output_states,
                              VolumeAir(states[2], model.P_a, model.muscle1),
                              VolumeAir(model.P_t, states[5], model.muscle2));
                }
                else if (mode == 2) {
                    Linearise(model,
                              states,
                              setpoint,
                              output_states,
                              VolumeAir(model.P_t, states[2], model.muscle1),
                              VolumeAir(states[5], model.P_a, model.muscle2));
                }
                else if (mode == 3) {
                    Linearise(model,
                              states,
                              setpoint,
                              output_states,
                              VolumeAir(model.P_t, states[2], model.muscle1),
                              VolumeAir(states[6], states[5], model.muscle2));
                }
                else {
                    Error_Handler();
                }

                return (ControlError(states, setpoint, output_states));
            }

            template <typename T>
            float VolumeAir(float P_1, float P_2, const T& m) {
                if (P_2 / P_1 > m.critical_ratio) {
                    return (P_1 * m.sonic_conductance * std::sqrt(m.T_0 / m.T_1)
                            * std::sqrt(1 - std::pow((P_2 / P_1 - m.critical_ratio) / (1 - m.critical_ratio), 2)));
                }
                return (P_1 * m.sonic_conductance * std::sqrt(m.T_0 / m.T_1));
            }

            template <typename T>
            void Linearise(const T& m,
                           std::vector<float>& states,
                           float setpoint,
                           std::vector<float>& output_states,
                           float dV_a1,
                           float dV_a2) {

                // TODO Populate these with the relevant functions
                // TODO NOTE None of this will work until i figure out the relevant states, the VolumeAIr state and
                // outputstate/cost might need to update
                float L_10 = m.muscle1.L_0;
                float L_20 = m.muscle2.L_0;
                float k_10 = m.muscle1.K_0;
                float k_20 = m.muscle2.K_0;
                float y    = states[0];
                float dy   = states[1];
                float P_m1 = states[2];
                float P_m2 = states[3];
                float P_a  = m.P_a;
                float a1   = m.muscle1.muscle_coefficients[0];
                float b1   = m.muscle1.muscle_coefficients[1];
                float c1   = m.muscle1.muscle_coefficients[2];
                float d1   = m.muscle1.muscle_coefficients[3];
                float a2   = m.muscle2.muscle_coefficients[0];
                float b2   = m.muscle2.muscle_coefficients[1];
                float c2   = m.muscle2.muscle_coefficients[2];
                float d2   = m.muscle2.muscle_coefficients[3];
                auto F_ce1 = m.muscle1.F_ce;  // TODO Make these eigen matrices
                auto F_ce2 = m.muscle2.F_ce;  // TODO Make these eigen matrices
                float R1   = m.muscle1.damping_coefficient;
                float R2   = m.muscle2.damping_coefficient;
                float mass = m.mass;

                // Column 1
                float Lin_mat_0_0 = y;
                float Lin_mat_1_0 = 0;
                float Lin_mat_2_0 = 0;
                float Lin_mat_3_0 = 0;

                // Column 2
                // Lin_mat_0_1
                // Calculate our first spring force
                Eigen::Matrix<float, 6, 1> P_ce1;

                P_ce1 << 1, P_m1, P_m1 * P_m1, P_m1 * P_m1 * P_m1, P_m1 * P_m1 * P_m1 * P_m1,
                    P_m1 * P_m1 * P_m1 * P_m1 * P_m1;

                float lam1 = k_10 + y / L_10;

                Eigen::Matrix<float, 1, 6> k_ce1;

                k_ce1 << 0, 1 / L_10, 2 * lam1 / L_10, 3 * lam1 * lam1 / L_10, 4 * lam1 * lam1 * lam1 / L_10,
                    5 * lam1 * lam1 * lam1 * lam1 / L_10;

                float F_s1 = k_ce1 * F_ce1 * P_ce1;

                // Calculate our second spring force
                Eigen::Matrix<float, 6, 1> P_ce2;

                P_ce2 << 1, P_m2, P_m2 * P_m2, P_m2 * P_m2 * P_m2, P_m2 * P_m2 * P_m2 * P_m2,
                    P_m2 * P_m2 * P_m2 * P_m2 * P_m2;

                float lam2 = k_20 - y / L_20;

                Eigen::Matrix<float, 1, 6> k_ce2;

                k_ce2 << 0, -1 / L_20, -2 * lam2 / L_20, -3 * lam2 * lam2 / L_20, -4 * lam2 * lam2 * lam2 / L_20,
                    -5 * lam2 * lam2 * lam2 * lam2 / L_20;

                float F_s2 = k_ce2 * F_ce2 * P_ce2;

                float Lin_mat_0_1 = (F_s2 - F_s1) / mass;

                float Lin_mat_1_1 = 0;

                // Lin_mat_2_1
                // Calculate our first spring force
                P_ce1 << 0, 1, 2 * P_m1, 3 * P_m1 * P_m1, 4 * P_m1 * P_m1 * P_m1, 5 * P_m1 * P_m1 * P_m1 * P_m1;

                lam1 = k_10 + y / L_10;

                k_ce1 << 1, lam1, lam1 * lam1, lam1 * lam1 * lam1, lam1 * lam1 * lam1 * lam1,
                    lam1 * lam1 * lam1 * lam1 * lam1;

                F_s1 = k_ce1 * F_ce1 * P_ce1;

                // Calculate our second spring force
                P_ce2 << 0, 1, 2 * P_m2, 3 * P_m2 * P_m2, 4 * P_m2 * P_m2 * P_m2, 5 * P_m2 * P_m2 * P_m2 * P_m2;

                lam2 = k_20 - y / L_20;

                k_ce2 << 1, lam2, lam2 * lam2, lam2 * lam2 * lam2, lam2 * lam2 * lam2 * lam2,
                    lam2 * lam2 * lam2 * lam2 * lam2;

                F_s2 = k_ce2 * F_ce2 * P_ce2;

                float F_d1 = -R1 / L_10;

                float Lin_mat_2_1 = (F_s2 - F_s1 - F_d1) / mass;

                // Lin_mat_3_1
                // Calculate our first spring force
                P_ce1 << 0, 1, 2 * P_m1, 3 * P_m1 * P_m1, 4 * P_m1 * P_m1 * P_m1, 5 * P_m1 * P_m1 * P_m1 * P_m1;

                lam1 = k_10 + y / L_10;

                k_ce1 << 1, lam1, lam1 * lam1, lam1 * lam1 * lam1, lam1 * lam1 * lam1 * lam1,
                    lam1 * lam1 * lam1 * lam1 * lam1;

                F_s1 = k_ce1 * F_ce1 * P_ce1;

                // Calculate our second spring force
                P_ce2 << 0, 1, 2 * P_m2, 3 * P_m2 * P_m2, 4 * P_m2 * P_m2 * P_m2, 5 * P_m2 * P_m2 * P_m2 * P_m2;

                lam2 = k_20 - y / L_20;

                k_ce2 << 1, lam2, lam2 * lam2, lam2 * lam2 * lam2, lam2 * lam2 * lam2 * lam2,
                    lam2 * lam2 * lam2 * lam2 * lam2;

                F_s2 = k_ce2 * F_ce2 * P_ce2;

                float F_d2 = -R2 / L_20;

                float Lin_mat_3_1 = (F_s2 - F_s1 + F_d2) / mass;

                // Column 3
                // Lin_mat_0_2
                lam1        = k_10 + y / L_10;
                float dV_m1 = 2 * b1 / (L_10 * L_10) + c1 / L_10 + 6 * a1 * lam1 / (L_10 * L_10);
                float V_m1  = c1 / L_10 + 3 * a1 * lam1 * lam1 / L_10 + 2 * b1 * lam1 / L_10;
                float V_a1  = V_m1 * P_m1 * P_a;

                float Lin_mat_0_2 = (P_a * dV_a1) / V_m1 - (P_m1 * dV_m1) / V_a1;

                float Lin_mat_1_2 = 0;
                float Lin_mat_2_2 = 0;
                float Lin_mat_3_2 = 0;

                // Column 4
                // Lin_mat_0_3
                lam2        = k_20 - y / L_20;
                float dV_m2 = 2 * b2 / (L_20 * L_20) + c2 / L_20 + 6 * a2 * lam2 / (L_20 * L_20);
                float V_m2  = c2 / L_20 + 3 * a2 * lam2 * lam2 / L_20 + 2 * b2 * lam2 / L_20;
                float V_a2  = V_m2 * P_m2 * P_a;

                float Lin_mat_0_3 = (P_a * dV_a2) / V_m2 - (P_m2 * dV_m2) / V_a2;

                float Lin_mat_1_3 = 0;
                float Lin_mat_2_3 = 0;
                float Lin_mat_3_3 = 0;

                // Calculate our next states
                output_states.push_back((Lin_mat_0_0 + Lin_mat_1_0 + Lin_mat_2_0 + Lin_mat_3_0) * y);
                output_states.push_back((Lin_mat_0_1 + Lin_mat_1_1 + Lin_mat_2_1 + Lin_mat_3_1) * dy);
                output_states.push_back((Lin_mat_0_2 + Lin_mat_1_2 + Lin_mat_2_2 + Lin_mat_3_2) * P_m1);
                output_states.push_back((Lin_mat_0_3 + Lin_mat_1_3 + Lin_mat_2_3 + Lin_mat_3_3) * P_m2);
            }

            std::pair<std::vector<float>, float> ControlError(std::vector<float>& states,
                                                              float setpoint,
                                                              std::vector<float>& output_states) {

                // Do the math to find the relative errors (state, input)
                std::vector<float> state_error = {output_states[0] - states[0],
                                                  output_states[1] - states[1],
                                                  output_states[2] - states[2],
                                                  output_states[3] - states[3]};
                float input_error              = output_states[0] - setpoint;
                return (std::make_pair(state_error, input_error));
            }

            template <typename T>
            std::pair<bool, bool> FirstLayer(const T& m, std::vector<float> states, float setpoint) {
                // Increment the depth (control horizon itt)

                ch_itt = 1;

                // Remove our previous results
                cost_result.clear();

                // Create a cost vector for each root
                std::vector<std::pair<std::vector<float>, float>> cost_root_1;
                std::vector<std::pair<std::vector<float>, float>> cost_root_2;
                std::vector<std::pair<std::vector<float>, float>> cost_root_3;

                // Create a output state vector for each process
                std::vector<float> output_states_1;
                std::vector<float> output_states_2;
                std::vector<float> output_states_3;

                // Calculate the result of performing each action and add the result error to the cost vector
                cost_root_1.push_back(ProcessModel(m, 1, states, setpoint, output_states_1));
                cost_root_2.push_back(ProcessModel(m, 2, states, setpoint, output_states_2));
                cost_root_3.push_back(ProcessModel(m, 3, states, setpoint, output_states_3));

                // Decide if the next layer is the last or not
                if (ch_itt >= ch_max - 1) {
                    // Must be on our last layer
                    FinalLayer(m, output_states_1, setpoint, 1, cost_root_1);
                    FinalLayer(m, output_states_2, setpoint, 2, cost_root_2);
                    FinalLayer(m, output_states_3, setpoint, 3, cost_root_3);
                }
                else {
                    // Not our last layer lets add a general layer
                    AddLayer(m, output_states_1, setpoint, 1, cost_root_1);
                    AddLayer(m, output_states_2, setpoint, 2, cost_root_2);
                    AddLayer(m, output_states_3, setpoint, 3, cost_root_3);
                }


                // Now look through the cost_result vector and pick the lowest cost to perform the root action
                auto result =
                    *std::min_element(cost_result.cbegin(), cost_result.cend(), [](const auto& lhs, const auto& rhs) {
                        return lhs.second < rhs.second;
                    });

                if (result.first == 1) {
                    utility::io::debug.out("Optimizer result mode1, %d\n", result.second);
                    return (std::make_pair(true, false));
                }
                else if (result.first == 2) {
                    utility::io::debug.out("Optimizer result mode2, %d\n", result.second);
                    return (std::make_pair(false, true));
                }
                else if (result.first == 3) {
                    utility::io::debug.out("Optimizer result mode3, %d\n", result.second);
                    return (std::make_pair(false, false));
                }
                utility::io::debug.out("Optimizer failed\n");
                return (std::make_pair(false, false));
            }

            template <typename T>
            void AddLayer(const T& m,
                          std::vector<float> states,
                          float setpoint,
                          int root,
                          std::vector<std::pair<std::vector<float>, float>>& cost) {
                // We're somewhere in the middle of our recursion
                ch_itt++;

                // Create a output state vector for each process
                std::vector<float> output_states_1;
                std::vector<float> output_states_2;
                std::vector<float> output_states_3;

                // Calculate the result of performing each action and add the result error to the cost vector
                cost.push_back(ProcessModel(m, 1, states, setpoint, output_states_1));
                cost.push_back(ProcessModel(m, 2, states, setpoint, output_states_2));
                cost.push_back(ProcessModel(m, 3, states, setpoint, output_states_3));

                // Decide if the next layer is the last or not
                if (ch_itt >= ch_max - 1) {
                    // Must be on our last layer
                    FinalLayer(m, output_states_1, setpoint, 1, cost);
                    FinalLayer(m, output_states_2, setpoint, 2, cost);
                    FinalLayer(m, output_states_3, setpoint, 3, cost);
                }
                else {
                    // Not our last layer lets add a general layer
                    AddLayer(m, output_states_1, setpoint, 1, cost);
                    AddLayer(m, output_states_2, setpoint, 2, cost);
                    AddLayer(m, output_states_3, setpoint, 3, cost);
                }

                // As we're leaving a level, decrement the itterator
                ch_itt--;
            }

            template <typename T>
            void FinalLayer(const T& m,
                            std::vector<float> states,
                            float setpoint,
                            int root,
                            std::vector<std::pair<std::vector<float>, float>>& cost) {
                // We're on or last layer, let's calculate the result append the cost and root

                // TODO This isn't needed on the last layer
                // Create a output state vector for each process
                std::vector<float> output_states_1;
                std::vector<float> output_states_2;
                std::vector<float> output_states_3;

                // Calculate the result of performing each action
                cost.push_back(ProcessModel(m, 1, states, setpoint, output_states_1));
                cost.push_back(ProcessModel(m, 2, states, setpoint, output_states_2));
                cost.push_back(ProcessModel(m, 3, states, setpoint, output_states_3));

                // Tally the cost and append it to the cost function list for each final cost
                auto state_error_sum = 0;
                auto input_error_sum = 0;
                for (auto& element : cost) {
                    state_error_sum = std::inner_product(state_weight.begin(),
                                                         state_weight.end(),
                                                         element.first.begin(),
                                                         state_error_sum,
                                                         std::plus<>(),
                                                         [](float a, float b) { return a * b * b; });

                    input_error_sum += input_weight * element.second * element.second;
                }

                cost_result.push_back(std::make_pair(root, state_error_sum + input_error_sum));
                cost_result.push_back(std::make_pair(root, state_error_sum + input_error_sum));
                cost_result.push_back(std::make_pair(root, state_error_sum + input_error_sum));
            }

        private:
            int ch_itt;
            const int ch_max;
            std::vector<std::pair<float, float>> cost_result;
            const std::vector<float> state_weight;
            const float input_weight;
        };

        extern Optimizer optimizer1;
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module

#endif  // MODULE_OPTIMIZER_HPP

// float L_10       = m.muscle1.L_0;
// float L_20       = m.muscle2.L_0;
// float k1_0       = m.muscle1.L_0;
// float k2_0       = m.muscle2.L_0;
// float y          = states[0];
// float vel        = states[1];
// float P_m1       = states[2];
// float P_m2       = states[3];
// float P_a        = m.P_a;
// float a1         = m.muscle1.muscle_coefficients[0];
// float b1         = m.muscle1.muscle_coefficients[1];
// float c1         = m.muscle1.muscle_coefficients[2];
// float d1         = m.muscle1.muscle_coefficients[3];
// float a2         = m.muscle2.muscle_coefficients[0];
// float b2         = m.muscle2.muscle_coefficients[1];
// float c2         = m.muscle2.muscle_coefficients[2];
// float d2         = m.muscle2.muscle_coefficients[3];
// float dV_A1dt    = V_a1;
// float dV_A2dt    = V_a2;
// float a_1_[6][6] = m.muscle1.F_ce;
// float a_2_[6][6] = m.muscle2.F_ce;
// float R1         = m.muscle1.damping_coefficient;
// float R2         = m.muscle2.damping_coefficient;
// float mass       = m.mass;

// // clang-format off
// float Lin_mat_0_0 = y;
// float Lin_mat_1_0 =
//     -(P_m1
//           * (a_1_[1][1] / L_10 + (2 * a_1_[2][1] * (y + L_10 * k1_0)) / std::pow(L_10, 2)
//              + (3 * a_1_[3][1] * std::pow((y + L_10 * k1_0), 2)) / std::pow(L_10, 3)
//              + (4 * a_1_[4][1] * std::pow((y + L_10 * k1_0), 3)) / std::pow(L_10, 4))
//       + P_m2
//             * (a_2_[1][1] / L_20 - (2 * a_2_[2][1] * (y - L_20 * k2_0)) / std::pow(L_20, 2)
//                + (3 * a_2_[3][1] * std::pow((y - L_20 * k2_0), 2)) / std::pow(L_20, 3)
//                - (4 * a_2_[4][1] * std::pow((y - L_20 * k2_0), 3)) / std::pow(L_20, 4))
//       + a_1_[1][0] / L_10 + a_2_[1][0] / L_20 + (std::pow(P_m1, 4) * a_1_[1][4]) / L_10
//       + (std::pow(P_m2, 4) * a_2_[1][4]) / L_20
//       + (2 * a_1_[2][0] * (y + L_10 * k1_0)) / std::pow(L_10, 2)
//       - (2 * a_2_[2][0] * (y - L_20 * k2_0)) / std::pow(L_20, 2)
//       + (std::pow(P_m1, 2)
//          * (3 * a_1_[3][2] * std::pow(L_10, 2) * std::pow(k1_0, 2)
//             + 2 * a_1_[2][2] * std::pow(L_10, 2) * k1_0 + a_1_[1][2] * std::pow(L_10, 2)
//             + 6 * a_1_[3][2] * L_10 * k1_0 * y + 2 * a_1_[2][2] * L_10 * y
//             + 3 * a_1_[3][2] * std::pow(y, 2)))
//             / std::pow(L_10, 3)
//       + (std::pow(P_m2, 2)
//          * (3 * a_2_[3][2] * std::pow(L_20, 2) * std::pow(k2_0, 2)
//             + 2 * a_2_[2][2] * std::pow(L_20, 2) * k2_0 + a_2_[1][2] * std::pow(L_20, 2)
//             - 6 * a_2_[3][2] * L_20 * k2_0 * y - 2 * a_2_[2][2] * L_20 * y
//             + 3 * a_2_[3][2] * std::pow(y, 2)))
//             / std::pow(L_20, 3)
//       + (3 * a_1_[3][0] * std::pow((y + L_10 * k1_0), 2)) / std::pow(L_10, 3)
//       + (4 * a_1_[4][0] * std::pow((y + L_10 * k1_0), 3)) / std::pow(L_10, 4)
//       + (5 * a_1_[5][0] * std::pow((y + L_10 * k1_0), 4)) / std::pow(L_10, 5)
//       + (3 * a_2_[3][0] * std::pow((y - L_20 * k2_0), 2)) / std::pow(L_20, 3)
//       - (4 * a_2_[4][0] * std::pow((y - L_20 * k2_0), 3)) / std::pow(L_20, 4)
//       + (5 * a_2_[5][0] * std::pow((y - L_20 * k2_0), 4)) / std::pow(L_20, 5)
//       + (std::pow(P_m1, 3) * (L_10 * a_1_[1][3] + 2 * a_1_[2][3] * y + 2 * L_10 * a_1_[2][3] * k1_0))
//             / std::pow(L_10, 2)
//       + (std::pow(P_m2, 3) * (L_20 * a_2_[1][3] - 2 * a_2_[2][3] * y + 2 * L_20 * a_2_[2][3] * k2_0))
//             / std::pow(L_20, 2))
//     / mass;
// float Lin_mat_2_0 =
//     (P_a
//      * (c1 * (k1_0 + y / L_10) + (3 * a1 * std::pow((k1_0 + y / L_10), 2)) / L_10
//         + (2 * b1 * (k1_0 + y / L_10)) / L_10)
//      * (c1 / L_10 + (3 * a1 * std::pow((k1_0 + y / L_10), 2)) / L_10
//         + (2 * b1 * (k1_0 + y / L_10)) / L_10))
//         / std::pow((d1 + c1 * (k1_0 + y / L_10) + a1 * std::pow((k1_0 + y / L_10), 3)
//                     + b1 * std::pow((k1_0 + y / L_10), 2)),
//                    2)
//     - (P_a
//        * ((2 * b1) / std::pow(L_10, 2) + c1 / L_10 + (6 * a1 * (k1_0 + y / L_10)) / std::pow(L_10,
//        2)))
//           / (d1 + c1 * (k1_0 + y / L_10) + a1 * std::pow((k1_0 + y / L_10), 3)
//              + b1 * std::pow((k1_0 + y / L_10), 2))
//     - (P_a * dV_A1dt
//        * (c1 / L_10 + (3 * a1 * std::pow((k1_0 + y / L_10), 2)) / L_10
//           + (2 * b1 * (k1_0 + y / L_10)) / L_10))
//           / std::pow((d1 + c1 * (k1_0 + y / L_10) + a1 * std::pow((k1_0 + y / L_10), 3)
//                       + b1 * std::pow((k1_0 + y / L_10), 2)),
//                      2);
// float Lin_mat_3_0 =
//     (P_a
//      * ((2 * b2) / std::pow(L_20, 2) + c2 / L_20 + (6 * a2 * (k2_0 - y / L_20)) / std::pow(L_20, 2)))
//         / (d2 + c2 * (k2_0 - y / L_20) + a2 * std::pow((k2_0 - y / L_20), 3)
//            + b2 * std::pow((k2_0 - y / L_20), 2))
//     - (P_a
//        * (c2 * (k2_0 - y / L_20) + (3 * a2 * std::pow((k2_0 - y / L_20), 2)) / L_20
//           + (2 * b2 * (k2_0 - y / L_20)) / L_20)
//        * (c2 / L_20 + (3 * a2 * std::pow((k2_0 - y / L_20), 2)) / L_20
//           + (2 * b2 * (k2_0 - y / L_20)) / L_20))
//           / std::pow((d2 + c2 * (k2_0 - y / L_20) + a2 * std::pow((k2_0 - y / L_20), 3)
//                       + b2 * std::pow((k2_0 - y / L_20), 2)),
//                      2)
//     + (P_a * dV_A2dt
//        * (c2 / L_20 + (3 * a2 * std::pow((k2_0 - y / L_20), 2)) / L_20
//           + (2 * b2 * (k2_0 - y / L_20)) / L_20))
//           / std::pow((d2 + c2 * (k2_0 - y / L_20) + a2 * std::pow((k2_0 - y / L_20), 3)
//                       + b2 * std::pow((k2_0 - y / L_20), 2)),
//                      2);
// float Lin_mat_0_1 = 0;
// float Lin_mat_1_1 = 0;
// float Lin_mat_2_1 = 0;
// float Lin_mat_3_1 = 0;
// float Lin_mat_0_2 = 0;
// float Lin_mat_1_2 =
//     -(a_1_[0][1] + a_1_[1][1] * (k1_0 + y / L_10)
//       + 3 * std::pow(P_m1, 2)
//             * (a_1_[0][3] + a_1_[1][3] * (k1_0 + y / L_10)
//                + a_1_[2][3] * std::pow((k1_0 + y / L_10), 2))
//       - R1 / L_10
//       + 2 * P_m1
//             * (a_1_[0][2] + a_1_[1][2] * (k1_0 + y / L_10) + a_1_[2][2] * std::pow((k1_0 + y / L_10),
//             2)
//                + a_1_[3][2] * std::pow((k1_0 + y / L_10), 3))
//       + a_1_[2][1] * std::pow((k1_0 + y / L_10), 2) + a_1_[3][1] * std::pow((k1_0 + y / L_10), 3)
//       + a_1_[4][1] * std::pow((k1_0 + y / L_10), 4) + 5 * std::pow(P_m1, 4) * a_1_[0][5]
//       + 4 * std::pow(P_m1, 3) * (a_1_[0][4] + a_1_[1][4] * (k1_0 + y / L_10)))
//     / mass;
// float Lin_mat_2_2 = 0;
// float Lin_mat_3_2 = 0;
// float Lin_mat_0_3 = 0;
// float Lin_mat_1_3 =
//     (a_2_[0][1] + a_2_[1][1] * (k2_0 - y / L_20)
//      + 3 * std::pow(P_m2, 2)
//            * (a_2_[0][3] + a_2_[1][3] * (k2_0 - y / L_20) + a_2_[2][3] * std::pow((k2_0 - y / L_20),
//            2))
//      - R2 / L_20
//      + 2 * P_m2
//            * (a_2_[0][2] + a_2_[1][2] * (k2_0 - y / L_20) + a_2_[2][2] * std::pow((k2_0 - y / L_20),
//            2)
//               + a_2_[3][2] * std::pow((k2_0 - y / L_20), 3))
//      + a_2_[2][1] * std::pow((k2_0 - y / L_20), 2) + a_2_[3][1] * std::pow((k2_0 - y / L_20), 3)
//      + a_2_[4][1] * std::pow((k2_0 - y / L_20), 4) + 5 * std::pow(P_m2, 4) * a_2_[0][5]
//      + 4 * std::pow(P_m2, 3) * (a_2_[0][4] + a_2_[1][4] * (k2_0 - y / L_20)))
//     / mass;
// float Lin_mat_2_3 = 0;
// float Lin_mat_3_3 = 0;
// clang-format on