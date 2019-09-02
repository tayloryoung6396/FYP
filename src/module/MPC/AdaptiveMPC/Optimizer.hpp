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
