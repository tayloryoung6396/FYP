// For each controller mode, recursively call the optimizer control horizon number of times
// Once a leg is complete, append to the cost list the cost value and the root action of the leg

#include "Optimizer.hpp"

Optimizer::Optimizer() {}

template <typename>
std::pair<std::vector<double>, double> Optimizer::ProcessModel(const T& model,
                                                               std::vector<double>& states,
                                                               double setpoint,
                                                               std::vector<double>& output_states) {

    // Given a control action, select the correct model matrix, from that linearize the model about the current state
    // point. Calculate the next state vector. Call the control error function and return the errors

    // TODO Find a way to know P1 and P2 values
    double Lin_mat[4][4];

    if (P2 / P1 > b) {
        model.A.Linearize(&Lin_mat);
    }
    else {
        model.B.Linearize(&Lin_mat);
    }

    // Calculate our next states
    output_states.push_back((Lin_mat[0][0] + Lin_mat[1][0] + Lin_mat[2][0] + Lin_mat[3][0]) * states[0]);
    output_states.push_back((Lin_mat[0][1] + Lin_mat[1][1] + Lin_mat[2][1] + Lin_mat[3][1]) * states[1]);
    output_states.push_back((Lin_mat[0][2] + Lin_mat[1][2] + Lin_mat[2][2] + Lin_mat[3][2]) * states[2]);
    output_states.push_back((Lin_mat[0][3] + Lin_mat[1][3] + Lin_mat[2][3] + Lin_mat[3][3]) * states[3]);

    return (ControlError(states, setpoint, output_states));
}

std::pair<std::vector<double>, double> Optimizer::ControlError(std::vector<double>& states,
                                                               double setpoint,
                                                               std::vector<double>& output_states) {

    // Do the math to find the relative errors (state, input)
    return (std::make_pair(output_states - states, output_state[0] - setpoint));
}

template <typename>
void Optimizer::FirstLayer(const T& model, std::vector<double>& states, double setpoint) {
    // Increment the depth (control horizon itt)
    Model m;

    ch_itt = 1;

    // Remove our previous results
    cost_result.clear();

    // Create a cost vector for each root
    std::vector<std::pair<std::vector<double>, double>> cost_root_1;
    std::vector<std::pair<std::vector<double>, double>> cost_root_2;
    std::vector<std::pair<std::vector<double>, double>> cost_root_3;

    // Create a output state vector for each process
    std::vector<double> output_states_1;
    std::vector<double> output_states_2;
    std::vector<double> output_states_3;

    // Calculate the result of performing each action and add the result error to the cost vector
    cost_root_1.push_back(ProcessModel(m.mode1, states, setpoint, output_states_1));
    cost_root_2.push_back(ProcessModel(m.mode2, states, setpoint, output_states_2));
    cost_root_3.push_back(ProcessModel(m.mode3, states, setpoint, output_states_3));

    // Decide if the next layer is the last or not
    if (ch_itt >= ch_max - 1) {
        // Must be on our last layer
        FinalLayer(m.mode1, output_states_1, setpoint, 1, cost_root_1);
        FinalLayer(m.mode2, output_states_2, setpoint, 2, cost_root_2);
        FinalLayer(m.mode3, output_states_3, setpoint, 3, cost_root_3);
    }
    else {
        // Not our last layer lets add a general layer
        AddLayer(m.mode1, output_states_1, setpoint, 1, cost_root_1);
        AddLayer(m.mode2, output_states_2, setpoint, 2, cost_root_2);
        AddLayer(m.mode3, output_states_3, setpoint, 3, cost_root_3);
    }


    // Now look through the cost_result vector and pick the lowest cost to perform the root action
    auto result = *std::min_element(cost_result.cbegin(), cost_result.cend(), [](const auto& lhs, const auto& rhs) {
        return lhs.second < rhs.second;
    });
    // std::cout << result.first << " " << result.second << std::endl;
    // root element => result.first
    // root cost    => result.second
}

void Optimizer::AddLayer(const T& model,
                         std::vector<double> states,
                         double setpoint,
                         int root,
                         std::vector<std::pair<double, double>>& cost) {
    // We're somewhere in the middle of our recursion
    ch_itt++;

    // Calculate the result of performing each action and add the result error to the cost vector
    cost.push_back(ProcessModel(m.mode1, states, setpoint, output_states_1));
    cost.push_back(ProcessModel(m.mode2, states, setpoint, output_states_2));
    cost.push_back(ProcessModel(m.mode3, states, setpoint, output_states_3));

    // Decide if the next layer is the last or not
    if (ch_itt >= ch_max - 1) {
        // Must be on our last layer
        FinalLayer(m.mode1, output_states_1, setpoint, 1, cost_root_1);
        FinalLayer(m.mode2, output_states_2, setpoint, 2, cost_root_2);
        FinalLayer(m.mode3, output_states_3, setpoint, 3, cost_root_3);
    }
    else {
        // Not our last layer lets add a general layer
        AddLayer(m.mode1, output_states_1, setpoint, 1, cost_root_1);
        AddLayer(m.mode2, output_states_2, setpoint, 2, cost_root_2);
        AddLayer(m.mode3, output_states_3, setpoint, 3, cost_root_3);
    }

    // As we're leaving a level, decrement the itterator
    ch_itt--;
}

void Optimizer::FinalLayer(const T& model,
                           std::vector<double> states,
                           double setpoint,
                           int root,
                           std::vector<std::pair<double, double>>& cost) {
    // We're on or last layer, let's calculate the result append the cost and root
    // Calculate the result of performing each action
    std::pair<std::vector<double>, double> cost_1 = ProcessModel(m.mode1, states, setpoint, output_states_1);
    std::pair<std::vector<double>, double> cost_2 = ProcessModel(m.mode2, states, setpoint, output_states_2);
    std::pair<std::vector<double>, double> cost_3 = ProcessModel(m.mode3, states, setpoint, output_states_3);

    // Tally the cost and append it to the cost function list for each final cost
    for (auto& element : cost) {
        state_error_sum += state_weight * element.first() ^ 2;
        input_error_sum += input_weight * element.second() ^ 2;
    }

    cost_result.push_back(std::make_pair(root,
                                         (state_error_sum + state_weight * cost_1.first() ^ 2)
                                             + (input_error_sum + input_weight * cost_1.second() ^ 2)));
    cost_result.push_back(std::make_pair(root,
                                         (state_error_sum + state_weight * cost_2.first() ^ 2)
                                             + (input_error_sum + input_weight * cost_2.second() ^ 2)));
    cost_result.push_back(std::make_pair(root,
                                         (state_error_sum + state_weight * cost_3.first() ^ 2)
                                             + (input_error_sum + input_weight * cost_3.second() ^ 2)));
}