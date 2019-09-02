#ifndef MODULE_OPTIMIZER_HPP
#define MODULE_OPTIMIZER_HPP

#include <vector>

namespace module {
namespace MPC {
    namespace AdaptiveMPC {
        class Optimizer {
        public:
            Optimizer(int ch_max, double state_weight, double input_weight);

            template <typename T>
            std::pair<std::vector<double>, double> ProcessModel(const T& model,
                                                                std::vector<double>& states,
                                                                double setpoint,
                                                                std::vector<double>& output_states);
            std::pair<std::vector<double>, double> ControlError(std::vector<double>& states,
                                                                double setpoint,
                                                                std::vector<double>& output_states);
            template <typename T>
            std::pair<bool, bool> FirstLayer(const T& m, std::vector<double>& states, double setpoint);
            template <typename T>
            void AddLayer(const T& m,
                          std::vector<double> states,
                          double setpoint,
                          int root,
                          std::vector<std::pair<double, double>>& cost);
            template <typename T>
            void FinalLayer(const T& m,
                            std::vector<double> states,
                            double setpoint,
                            int root,
                            std::vector<std::pair<double, double>>& cost);

        private:
            static int ch_itt;
            const int ch_max;
            std::vector<std::pair<double, double>> cost_result;
            const double state_weight;
            const double input_weight;
        };
    }  // namespace AdaptiveMPC
}  // namespace MPC
}  // namespace module

#endif  // MODULE_OPTIMIZER_HPP
