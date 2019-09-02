// For each controller mode, recursively call the optimizer control horizon number of times
// Once a leg is complete, append to the cost list the cost value and the root action of the leg

#ifndef MODULE_OPTIMIZER_HPP
#define MODULE_OPTIMIZER_HPP

#include <vector>

namespace module {
namespace MPC {
    namespace AdaptiveMPC {
        class Optimizer {
        public:
            void FirstLayer();
            void AddLayer(int root, std::vector<std::pair<double, double>>& cost);
            void FinalLayer(int root, std::vector<std::pair<double, double>>& cost);
            std::pair<double, double> ControlError();

        private:
            static int ch_itt = 0;
            const int ch_max  = 5;
            std::vector<std::pair<double, double>> cost_result;
        };
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module

#endif  // MODULE_OPTIMIZER_HPP


#include "Optimizer.hpp"

// This doesn't live here
template <typename>
void Optimizer::Init(const T& model, std::vector<double> states, double setpoint) {
    // Setup our optimizer to use the model given
    // Initialise the setpoint and states to the given values

    FirstLayer(states);
}

std::pair<double, double> Optimizer::ProcessModel() {

    // Given a control action, select the correct model matrix, from that linearize the model about the current state
    // point. Calculate the next state vector. Call the control error function and return the errors

    model.Linearize();

    return (ControlError());
}

std::pair<double, double> Optimizer::ControlError() {

    // Do the math to find the relative errors
    return (std::make_pair(state_error, input_error));
}

void Optimizer::FirstLayer() {
    // Increment the depth (control horizon itt)
    ch_itt = 1;

    // Remove our previous results
    cost_result.clear();

    // Create a cost vector for each root
    std::vector<std::pair<double, double>> cost_root_1;
    std::vector<std::pair<double, double>> cost_root_2;
    std::vector<std::pair<double, double>> cost_root_3;

    // Calculate the result of performing each action and add the result error to the cost vector
    // TODO This is wrong
    cost_root_1.push_back(ProcessModel());
    cost_root_2.push_back(ProcessModel());
    cost_root_3.push_back(ProcessModel());

    // Decide if the next layer is the last or not
    if (ch_itt >= ch_max - 1) {
        // Must be on our last layer
        FinalLayer(1, cost_root_1);
        FinalLayer(2, cost_root_2);
        FinalLayer(3, cost_root_3);
    }
    else {
        // Not our last layer lets add a general layer
        AddLayer(1, cost_root_1);
        AddLayer(2, cost_root_2);
        AddLayer(3, cost_root_3);
    }


    // Now look through the cost_result vector and pick the lowest cost to perform the root action
    auto result = *std::min_element(cost_result.cbegin(), cost_result.cend(), [](const auto& lhs, const auto& rhs) {
        return lhs.second < rhs.second;
    });
    // std::cout << result.first << " " << result.second << std::endl;
    // root element => result.first
    // root cost    => result.second
}

void Optimizer::AddLayer(int root, std::vector<std::pair<double, double>>& cost) {
    // We're somewhere in the middle of our recursion
    ch_itt++;

    // Calculate the result of performing the first action and add the result error to the cost vector
    // TODO This is wrong
    cost.push_back(ProcessModel());
    // Calculate the result of performing the second action and add the result error to the cost vector
    // TODO This is wrong
    cost.push_back(ProcessModel());
    // Calculate the result of performing the third action and add the result error to the cost vector
    // TODO This is wrong
    cost.push_back(ProcessModel());

    // Decide if the next layer is the last or not
    if (ch_itt >= ch_max - 1) {
        // Must be on our last layer
        FinalLayer(1, cost);
        FinalLayer(2, cost);
        FinalLayer(3, cost);
    }
    else {
        // Not our last layer lets add a general layer
        AddLayer(1, cost);
        AddLayer(2, cost);
        AddLayer(3, cost);
    }

    // As we're leaving a level, decrement the itterator
    ch_itt--;
}

void Optimizer::FinalLayer(int root, std::vector<std::pair<double, double>>& cost) {
    // We're on or last layer, let's calculate the result append the cost and root

    // Calculate the result of performing each action
    std::pair<double, double> cost_1 = ProcessModel();
    std::pair<double, double> cost_2 = ProcessModel();
    std::pair<double, double> cost_3 = ProcessModel();

    // Tally the cost and append it to the cost function list for each final cost
    for (auto& element : cost) {
        state_error += state_weight * element.first() ^ 2;
        input_error += input_weight * element.second() ^ 2;
    }

    cost_result.push_back(std::make_pair(
        root, (state_error + state_weight * cost_1.first() ^ 2) + (input_error + input_weight * cost_1.second() ^ 2)));
    cost_result.push_back(std::make_pair(
        root, (state_error + state_weight * cost_2.first() ^ 2) + (input_error + input_weight * cost_2.second() ^ 2)));
    cost_result.push_back(std::make_pair(
        root, (state_error + state_weight * cost_3.first() ^ 2) + (input_error + input_weight * cost_3.second() ^ 2)));
}