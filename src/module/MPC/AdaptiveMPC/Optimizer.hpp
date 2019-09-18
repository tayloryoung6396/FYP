#ifndef MODULE_OPTIMIZER_HPP
#define MODULE_OPTIMIZER_HPP

#include <algorithm>
#include <functional>
#include <numeric>
#include <vector>
#include "MPC/Model/Model.hpp"
#include "utility/io/uart.hpp"

namespace module {
namespace MPC {
    namespace AdaptiveMPC {
        class Optimizer {
        public:
            Optimizer(int ch_max, float state_weight, float input_weight);

            template <typename T>
            std::pair<std::vector<float>, float> ProcessModel(const T& model,
                                                              std::vector<float>& states,
                                                              float setpoint,
                                                              std::vector<float>& output_states) {

                // Given a control action, select the correct model matrix, from that linearize the model about the
                // current state point. Calculate the next state vector. Call the control error function and return the
                // errors

                // TODO Find a way to know P1 and P2 values
                float Lin_mat[4][4];

                // if (P2 / P1 > b) {
                //     Lin_mat = model.A1.Linearize();
                // }
                // else {
                //     Lin_mat = model.A2.Linearize();
                // }

                // Calculate our next states
                output_states.push_back((Lin_mat[0][0] + Lin_mat[1][0] + Lin_mat[2][0] + Lin_mat[3][0]) * states[0]);
                output_states.push_back((Lin_mat[0][1] + Lin_mat[1][1] + Lin_mat[2][1] + Lin_mat[3][1]) * states[1]);
                output_states.push_back((Lin_mat[0][2] + Lin_mat[1][2] + Lin_mat[2][2] + Lin_mat[3][2]) * states[2]);
                output_states.push_back((Lin_mat[0][3] + Lin_mat[1][3] + Lin_mat[2][3] + Lin_mat[3][3]) * states[3]);

                return (ControlError(states, setpoint, output_states));
            }

            std::pair<std::vector<float>, float> ControlError(std::vector<float>& states,
                                                              float setpoint,
                                                              std::vector<float>& output_states) {

                // Do the math to find the relative errors (state, input)
                std::vector<float> state_error = {output_states[0] - states[0],
                                                  output_states[1] - states[1],
                                                  output_states[2] - states[2],
                                                  output_states[2] - states[2]};
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
                cost_root_1.push_back(ProcessModel(m.mode1, states, setpoint, output_states_1));
                cost_root_2.push_back(ProcessModel(m.mode2, states, setpoint, output_states_2));
                cost_root_3.push_back(ProcessModel(m.mode3, states, setpoint, output_states_3));

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
                cost.push_back(ProcessModel(m.mode1, states, setpoint, output_states_1));
                cost.push_back(ProcessModel(m.mode2, states, setpoint, output_states_2));
                cost.push_back(ProcessModel(m.mode3, states, setpoint, output_states_3));

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
                cost.push_back(ProcessModel(m.mode1, states, setpoint, output_states_1));
                cost.push_back(ProcessModel(m.mode2, states, setpoint, output_states_2));
                cost.push_back(ProcessModel(m.mode3, states, setpoint, output_states_3));

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
