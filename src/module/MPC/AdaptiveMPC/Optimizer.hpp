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
            Optimizer(int ch_max);

            template <typename T>
            float ProcessModel(const T& model,
                               int mode,
                               std::vector<float>& states,
                               std::vector<float>& setpoint,
                               std::vector<float>& input_states,
                               std::vector<float>& output_states) {

                utility::io::debug.out("Process model mode %d\n", mode);

                // Given a control action, select the correct model matrix, from that linearize the model about the
                // current state point. Calculate the next state vector. Call the control error function and return the
                // errors
                Eigen::Matrix<float, 4, 4> A_mat;
                Eigen::Matrix<float, 4, 4> I;
                /* clang-format off */
                I << 1, 0, 0, 0, 
                     0, 1, 0, 0,
                     0, 0, 1, 0,
                     0, 0, 0, 1;
                /* clang-format on */
                if (mode == 1) {
                    Linearise(model,
                              states,
                              dotV_a(states[2], model.P_a, model.muscle1),
                              dotV_a_P_m1(states[2], model.P_a, model.muscle1, mode),
                              dotV_a(model.P_t, states[3], model.muscle2),
                              dotV_a_P_m2(model.P_t, states[3], model.muscle2, mode),
                              A_mat);
                }
                else if (mode == 2) {
                    Linearise(model,
                              states,
                              dotV_a(model.P_t, states[2], model.muscle1),
                              dotV_a_P_m1(model.P_t, states[2], model.muscle1, mode),
                              dotV_a(states[3], model.P_a, model.muscle2),
                              dotV_a_P_m2(states[3], model.P_a, model.muscle2, mode),
                              A_mat);
                }
                else if (mode == 3) {
                    Linearise(model,
                              states,
                              dotV_a(model.P_t, states[2], model.muscle1),
                              dotV_a_P_m1(model.P_t, states[2], model.muscle1, mode),
                              dotV_a(model.P_t, states[3], model.muscle2),
                              dotV_a_P_m2(model.P_t, states[3], model.muscle2, mode),
                              A_mat);
                }
                else {
                    Error_Handler();
                }

                float Sampling_time2 = 50;  // 0.01 T_s

                Eigen::Matrix<float, 4, 1> x_state;
                x_state << input_states[0], input_states[1], input_states[2], input_states[3];

                Eigen::Matrix<float, 4, 4> A_mat_Ts(Sampling_time2 * A_mat);
                Eigen::Matrix<float, 4, 4> A_mat_k(I + A_mat_Ts);
                Eigen::Matrix<float, 4, 1> x_states_update(A_mat_k * x_state);

                output_states.push_back(x_states_update[0]);
                output_states.push_back(x_states_update[1]);
                output_states.push_back(x_states_update[2]);
                output_states.push_back(x_states_update[3]);

                return (ControlError(setpoint, output_states));
            }

            template <typename T>
            float dotV_a(float P_1, float P_2, const T& m) {
                float b = m.critical_ratio;
                float C = m.sonic_conductance;
                float A = C * std::sqrt(m.T_0 / m.T_1);
                if (P_2 / P_1 > b) {
                    return (P_1 * A * std::sqrt(1 - std::pow((P_2 / P_1 - b) / (1 - b), 2)));
                }
                return (P_1 * A);
            }

            template <typename T>
            float dotV_a_P_m1(float P_1, float P_2, const T& m, int mode) {
                float b = m.critical_ratio;
                float C = m.sonic_conductance;
                float A = C * std::sqrt(m.T_0 / m.T_1);
                if (mode == (2 || 3)) {
                    // P_2 == P_m1
                    if (P_2 / P_1 > b) {
                        return -(A * (P_2 / P_1 - b)
                                 / ((1 - b) * (1 - b) * std::sqrt(1 - std::pow((P_2 / P_1 - b) / (1 - b), 2))));
                    }
                    return 0;
                }
                else {
                    // P_1 == P_m1
                    if (P_2 / P_1 > b) {
                        return A * std::sqrt(1 - std::pow((P_2 / P_1 - b) / (1 - b), 2))
                               + (A * P_2 * (P_2 / P_1 - b)
                                  / ((1 - b) * (1 - b) * P_1 * std::sqrt(1 - std::pow((P_2 / P_1 - b) / (1 - b), 2))));
                    }

                    return C * std::sqrt(m.T_0 / m.T_1);
                }
                return 0;
            }

            template <typename T>
            float dotV_a_P_m2(float P_1, float P_2, const T& m, int mode) {
                float b = m.critical_ratio;
                float C = m.sonic_conductance;
                float A = C * std::sqrt(m.T_0 / m.T_1);
                if (mode == (1 || 3)) {
                    // P_2 == P_m2
                    if (P_2 / P_1 > b) {
                        return -(A * (P_2 / P_1 - b)
                                 / ((1 - b) * (1 - b) * std::sqrt(1 - std::pow((P_2 / P_1 - b) / (1 - b), 2))));
                    }
                    return 0;
                }
                else {
                    // P_1 == P_m2
                    if (P_2 / P_1 > b) {
                        return A * std::sqrt(1 - std::pow((P_2 / P_1 - b) / (1 - b), 2))
                               + (A * P_2 * (P_2 / P_1 - b)
                                  / ((1 - b) * (1 - b) * P_1 * std::sqrt(1 - std::pow((P_2 / P_1 - b) / (1 - b), 2))));
                    }

                    return C * std::sqrt(m.T_0 / m.T_1);
                }
                return 0;
            }

            template <typename T>
            void Linearise(const T& m,
                           std::vector<float>& states,
                           float dotV_a1,
                           float dotV_a2,
                           float ddotV_a1,
                           float ddotV_a2,
                           Eigen::Matrix<float, 4, 4> A_mat) {

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
                auto F_ce1 = m.muscle1.F_ce;
                auto F_ce2 = m.muscle2.F_ce;
                float R1   = m.muscle1.damping_coefficient;
                float R2   = m.muscle2.damping_coefficient;
                float mass = m.mass;

                Eigen::Matrix<float, 6, 1> P_ce1;
                Eigen::Matrix<float, 6, 1> P_ce2;
                Eigen::Matrix<float, 6, 1> dP_ce1;
                Eigen::Matrix<float, 6, 1> dP_ce2;

                Eigen::Matrix<float, 1, 6> k_ce1;
                Eigen::Matrix<float, 1, 6> k_ce2;
                Eigen::Matrix<float, 1, 6> dk_ce1;
                Eigen::Matrix<float, 1, 6> dk_ce2;

                float F_s1;
                float F_s2;
                float F_d1;
                float F_d2;
                float V_m1;
                float V_m2;
                float dV_m1;
                float dV_m2;
                float dotV_m1;
                float dotV_m2;
                float ddotV_m1;
                float ddotV_m2;
                float ddotV_m1_dV_m1;
                float ddotV_m2_dV_m2;

                /* clang-format off */

                float k1    = k_10 + y / L_10;
                float k2    = k_20 - y / L_20;
                float dotk1 = k_10 + dy / L_10;
                float dotk2 = k_20 - dy / L_20;

                P_ce1 << 1, 
                         P_m1, 
                         (P_m1 * P_m1), 
                         (P_m1 * P_m1 * P_m1), 
                         (P_m1 * P_m1 * P_m1 * P_m1), 
                         (P_m1 * P_m1 * P_m1 * P_m1 * P_m1);
                P_ce2 << 1, 
                         P_m2, 
                         (P_m2 * P_m2), 
                         (P_m2 * P_m2 * P_m2), 
                         (P_m2 * P_m2 * P_m2 * P_m2), 
                         (P_m2 * P_m2 * P_m2 * P_m2 * P_m2);
                dP_ce1 << 0, 
                          1, 
                          (2 * P_m1), 
                          (3 * P_m1 * P_m1), 
                          (4 * P_m1 * P_m1 * P_m1), 
                          (5 * P_m1 * P_m1 * P_m1 * P_m1);
                dP_ce2 << 0, 
                          1, 
                          (2 * P_m2), 
                          (3 * P_m2 * P_m2), 
                          (4 * P_m2 * P_m2 * P_m2),
                          (5 * P_m2 * P_m2 * P_m2 * P_m2);


                k_ce1 << 1, 
                         k1, 
                         (k1 * k1), 
                         (k1 * k1 * k1), 
                         (k1 * k1 * k1 * k1), 
                         (k1 * k1 * k1 * k1 * k1);
                k_ce2 << 1, 
                         k2, 
                         (k2 * k2), 
                         (k2 * k2 * k2), 
                         (k2 * k2 * k2 * k2), 
                         (k2 * k2 * k2 * k2 * k2);
                dk_ce1 << 0, 
                          (1 / L_10), 
                          (2 * k1 / L_10), 
                          (3 * k1 * k1 / L_10), 
                          (4 * k1 * k1 * k1 / L_10),
                          (5 * k1 * k1 * k1 * k1 / L_10);
                dk_ce2 << 0, 
                          (-1 / L_20),
                          (-2 * k2 / L_20), 
                          (-3 * k2 * k2 / L_20),
                          (-4 * k2 * k2 * k2 / L_20),
                          (-5 * k2 * k2 * k2 * k2 / L_20);

                /******************************************************************************************************/
                /********************************************** Column 1 **********************************************/
                /******************************************************************************************************/

                /********************************************* Lin_mat_0_0 ********************************************/
                float Lin_mat_0_0 = 0;
                /********************************************* Lin_mat_1_0 ********************************************/

                // Calculate our first spring force
                F_s1 = dk_ce1 * F_ce1 * P_ce1;

                // Calculate our second spring force
                F_s2 = dk_ce2 * F_ce2 * P_ce2;

                float Lin_mat_1_0 = (F_s2 - F_s1) / mass;

                /********************************************* Lin_mat_2_0 ********************************************/
                dV_m1          = (3 * a1 * k1 * k1 + 2 * b1 * k1 + c1) / L_10;
                ddotV_m1_dV_m1 =
                    ((a1 * k1 * k1 * k1 + b1 * k1 * k1 + c1 * k1 + d1) 
                        * (6 * a1 * k1 * dotk1 + 2 * b1 * dotk1)
                    - (3 * a1 * k1 * k1 * dotk1 + 2 * b1 * k1 * dotk1 + c1 * dotk1) 
                        * (3 * a1 * k1 * k1 + 2 * b1 * k1 + c1))
                    / (L_10 * (a1 * k1 * k1 * k1 + b1 * k1 * k1 + c1 * k1 + d1)
                       * (a1 * k1 * k1 * k1 + b1 * k1 * k1 + c1 * k1 + d1));

                float Lin_mat_2_0 = (P_a * dotV_a1 / dV_m1) - (P_m1 * ddotV_m1_dV_m1);
                /********************************************* Lin_mat_3_0 ********************************************/
                dV_m2          = (3 * a2 * k2 * k2 + 2 * b2 * k2 + c2) / L_20;
                ddotV_m2_dV_m2 =
                    ((a2 * k2 * k2 * k2 + b2 * k2 * k2 + c2 * k2 + d2) 
                        * (6 * a2 * k2 * dotk2 + 2 * b2 * dotk2)
                    - (3 * a2 * k2 * k2 * dotk2 + 2 * b2 * k2 * dotk2 + c2 * dotk2) 
                        * (3 * a2 * k2 * k2 + 2 * b2 * k2 + c2))
                    / (L_20 * (a2 * k2 * k2 * k2 + b2 * k2 * k2 + c2 * k2 + d2)
                       * (a2 * k2 * k2 * k2 + b2 * k2 * k2 + c2 * k2 + d2));

                float Lin_mat_3_0 = (P_a * dotV_a2 / dV_m2) - (P_m2 * ddotV_m2_dV_m2);
                /******************************************************************************************************/
                /********************************************** Column 2 **********************************************/
                /******************************************************************************************************/

                /********************************************* Lin_mat_0_1 ********************************************/
                float Lin_mat_0_1 = 1;
                /********************************************* Lin_mat_1_1 ********************************************/
                F_d1 = -R1 * P_m1 / L_10;
                F_d2 = -R2 * P_m2 / L_20;

                float Lin_mat_1_1 = (F_d2 - F_d1) / mass;
                /********************************************* Lin_mat_2_1 ********************************************/
                V_m1     = (a1 * k1 * k1 * k1) + (b1 * k1 * k1) + (c1 * k1) + d1;
                ddotV_m1 = (3 * a1 * k1 * k1 + 2 * b1 * k1 + c1) / L_10;

                float Lin_mat_2_1 = -P_m1 * ddotV_m1 / V_m1;
                /********************************************* Lin_mat_3_1 ********************************************/
                V_m2     = (a2 * k2 * k2 * k2) + (b2 * k2 * k2) + (c2 * k2) + d2;
                ddotV_m2 = (3 * a2 * k2 * k2 + 2 * b2 * k2 + c2) / L_20;

                float Lin_mat_3_1 = -P_m2 * ddotV_m2 / V_m2;
                /******************************************************************************************************/
                /********************************************** Column 3 **********************************************/
                /******************************************************************************************************/

                /********************************************* Lin_mat_0_2 ********************************************/
                float Lin_mat_0_2 = 0;
                /********************************************* Lin_mat_1_2 ********************************************/
                F_s1 = k_ce1 * F_ce1 * dP_ce1;
                F_d1 = -R1 * dotk1;

                float Lin_mat_1_2 = (-F_s1 - F_d1) / mass;
                /********************************************* Lin_mat_2_2 ********************************************/
                dotV_m1 = (3 * a1 * k1 * k1 * dotk1 + 2 * b1 * k1 * dotk1 + c1 * dotk1) ;
                V_m1    = (a1 * k1 * k1 * k1) + (b1 * k1 * k1) + (c1 * k1) + d1;;
                
                float Lin_mat_2_2 = (P_a * ddotV_a1 - dotV_m1) / V_m1;
                /********************************************* Lin_mat_3_2 ********************************************/
                float Lin_mat_3_2 = 0;
                /******************************************************************************************************/
                /********************************************** Column 4 **********************************************/
                /******************************************************************************************************/

                /********************************************* Lin_mat_0_3 ********************************************/
                float Lin_mat_0_3 = 0;
                /********************************************* Lin_mat_1_3 ********************************************/
                F_s2 = k_ce2 * F_ce2 * dP_ce2;
                F_d2 = -R2 * dotk2;

                float Lin_mat_1_3 = (F_s2 + F_d2) / mass;
                /********************************************* Lin_mat_2_3 ********************************************/
                float Lin_mat_2_3 = 0;
                /********************************************* Lin_mat_3_3 ********************************************/
                dotV_m2 = (3 * a2 * k2 * k2 * dotk2 + 2 * b2 * k2 * dotk2 + c2 * dotk2) ;
                V_m2    = (a2 * k2 * k2 * k2) + (b2 * k2 * k2) + (c2 * k2) + d2;
                
                float Lin_mat_3_3 = (P_a * ddotV_a2 - dotV_m2) / V_m2;

                A_mat << Lin_mat_0_0, Lin_mat_0_1, Lin_mat_0_2, Lin_mat_0_3, 
                         Lin_mat_1_0, Lin_mat_1_1, Lin_mat_1_2, Lin_mat_1_3, 
                         Lin_mat_2_0, Lin_mat_2_1, Lin_mat_2_2, Lin_mat_2_3, 
                         Lin_mat_3_0, Lin_mat_3_1, Lin_mat_3_2, Lin_mat_3_3;
                /* clang-format on */

                utility::io::debug.out("Linearised mat\n");
                utility::io::debug.out("%.2f %.2f %.2f %.2f\n", Lin_mat_0_0, Lin_mat_0_1, Lin_mat_0_2, Lin_mat_0_3);
                utility::io::debug.out("%.2f %.2f %.2f %.2f\n", Lin_mat_1_0, Lin_mat_1_1, Lin_mat_1_2, Lin_mat_1_3);
                utility::io::debug.out("%.2f %.2f %.2f %.2f\n", Lin_mat_2_0, Lin_mat_2_1, Lin_mat_2_2, Lin_mat_2_3);
                utility::io::debug.out("%.2f %.2f %.2f %.2f\n", Lin_mat_3_0, Lin_mat_3_1, Lin_mat_3_2, Lin_mat_3_3);
            }

            float ControlError(std::vector<float>& setpoint, std::vector<float>& output_states) {

                // Do the math to find the relative errors (state, input)
                std::vector<float> state_error = {output_states[0] - setpoint[0],
                                                  output_states[1] - setpoint[1],
                                                  output_states[2] - setpoint[2],
                                                  output_states[3] - setpoint[3]};

                for (std::vector<float>::const_iterator i = state_error.begin(); i != state_error.end(); ++i) {
                    utility::io::debug.out("%f ", *i);
                }
                utility::io::debug.out("\n");
                // TODO Fix
                return (0);  // state_error.transpose() * state_weight * state_error);
            }

            template <typename T>
            std::pair<bool, bool> FirstLayer(const T& m, std::vector<float>& states, std::vector<float>& setpoint) {
                // Increment the depth (control horizon itt)
                for (std::vector<float>::const_iterator i = states.begin(); i != states.end(); ++i) {
                    utility::io::debug.out("%f ", *i);
                }
                utility::io::debug.out("\n");

                ch_itt = 1;

                // Remove our previous results
                cost_result.clear();

                // Create a cost for each root
                float cost_root_1;
                float cost_root_2;
                float cost_root_3;

                // Create a output state vector for each process
                std::vector<float> output_states_1;
                std::vector<float> output_states_2;
                std::vector<float> output_states_3;

                // Calculate the result of performing each action and add the result error to the cost vector
                cost_root_1 = ProcessModel(m, 1, states, setpoint, states, output_states_1);
                cost_root_2 = ProcessModel(m, 2, states, setpoint, states, output_states_2);
                cost_root_3 = ProcessModel(m, 3, states, setpoint, states, output_states_3);

                // Decide if the next layer is the last or not
                if (ch_itt >= ch_max - 1) {
                    // Must be on our last layer
                    FinalLayer(m, states, setpoint, output_states_1, 1, cost_root_1);
                    FinalLayer(m, states, setpoint, output_states_2, 2, cost_root_2);
                    FinalLayer(m, states, setpoint, output_states_3, 3, cost_root_3);
                }
                else {
                    // Not our last layer lets add a general layer
                    AddLayer(m, states, setpoint, output_states_1, 1, cost_root_1);
                    AddLayer(m, states, setpoint, output_states_2, 2, cost_root_2);
                    AddLayer(m, states, setpoint, output_states_3, 3, cost_root_3);
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
                          std::vector<float>& states,
                          std::vector<float>& setpoint,
                          std::vector<float>& input_states,
                          int root,
                          float cost) {
                // We're somewhere in the middle of our recursion
                ch_itt += 1;

                // Create a output state vector for each process
                std::vector<float> output_states_1;
                std::vector<float> output_states_2;
                std::vector<float> output_states_3;

                // Calculate the result of performing each action and add the result error to the cost vector
                float cost1 = ProcessModel(m, 1, states, setpoint, input_states, output_states_1);
                float cost2 = ProcessModel(m, 2, states, setpoint, input_states, output_states_2);
                float cost3 = ProcessModel(m, 3, states, setpoint, input_states, output_states_3);

                // Decide if the next layer is the last or not
                if (ch_itt >= ch_max - 1) {
                    // Must be on our last layer
                    FinalLayer(m, states, setpoint, output_states_1, 1, cost1);
                    FinalLayer(m, states, setpoint, output_states_2, 2, cost2);
                    FinalLayer(m, states, setpoint, output_states_3, 3, cost3);
                }
                else {
                    // Not our last layer lets add a general layer
                    AddLayer(m, states, setpoint, output_states_1, 1, cost1);
                    AddLayer(m, states, setpoint, output_states_2, 2, cost2);
                    AddLayer(m, states, setpoint, output_states_3, 3, cost3);
                }

                // As we're leaving a level, decrement the itterator
                ch_itt -= 1;
            }

            template <typename T>
            void FinalLayer(const T& m,
                            std::vector<float>& states,
                            std::vector<float>& setpoint,
                            std::vector<float>& input_states,
                            int root,
                            float& cost) {
                // We're on or last layer, let's calculate the result append the cost and root

                // TODO This isn't needed on the last layer
                // Create a output state vector for each process
                std::vector<float> output_states_1;
                std::vector<float> output_states_2;
                std::vector<float> output_states_3;

                // Calculate the result of performing each action and add the result error to the cost vector
                float cost1 = ProcessModel(m, 1, states, setpoint, input_states, output_states_1);
                float cost2 = ProcessModel(m, 2, states, setpoint, input_states, output_states_2);
                float cost3 = ProcessModel(m, 3, states, setpoint, input_states, output_states_3);

                // Tally the cost and append it to the cost function list for each final cost
                cost_result.push_back(std::make_pair(cost + cost1, 1));
                cost_result.push_back(std::make_pair(cost + cost2, 2));
                cost_result.push_back(std::make_pair(cost + cost3, 3));
            }

        private:
            int ch_itt;
            const int ch_max;
            std::vector<std::pair<float, float>> cost_result;
            Eigen::Matrix<float, 4, 4> state_weight;
        };

        extern Optimizer optimizer1;
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module

#endif  // MODULE_OPTIMIZER_HPP