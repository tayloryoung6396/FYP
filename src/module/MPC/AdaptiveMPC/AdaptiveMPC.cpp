// Take our setpoint input
// u = theta

// transfer it to a y length value through the model function
// y = model(u)

// get our system measured variables vector
// std::vector things

// Call the optimizer with the model info passed
// struct Model_A {
//     void compute();
//     int value;
// };

// template <typename>
// void Optimizer::process_model(const T& model) {
//     model.compute();
// }

#include "AdaptiveMPC.hpp"
#include "Optimizer.hpp"

AdaptiveMPC::AdaptiveMPC() {}

void AdaptiveMPC::Compute() {

    dynamic_model m;

    // Convert the input setpoint to the y length setpoint
    // setpoint = Model::Something(theta);

    // Get our measured state vector
    // std::vector<double> states = TODO

    void Optimizer::FirstLayer(m, states, setpoint);
}