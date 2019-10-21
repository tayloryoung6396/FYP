#ifndef MODULE_ONE_AXIS_HPP
#define MODULE_ONE_AXIS_HPP

#define _USE_MATH_DEFINES
#define M_PI 3.14

#include <vector>
#include "HardwareIO/Muscle/Muscle.hpp"
#include "MPC/AdaptiveMPC/AdaptiveMPC.hpp"

namespace module {
namespace HardwareIO {
    namespace joint {

        struct axis_model_t {
            float mass;
            float radius;
            float P_a;
            float P_t;
            std::pair<float, float> limits;
            module::HardwareIO::muscle_properties_t muscle1;
            module::HardwareIO::muscle_properties_t muscle2;
        };

        template <typename T>
        class OneAxis {
        public:
            OneAxis(std::vector<module::HardwareIO::muscle_t>& muscle,
                    float mass,
                    float radius,
                    T controller,
                    module::MPC::AdaptiveMPC::Optimizer optimizer)
                : muscle1(muscle[0]), muscle2(muscle[1]), controller(optimizer) {
                axis_model.mass    = mass;
                axis_model.radius  = radius;
                axis_model.P_a     = 101325;  // 1 atm
                axis_model.muscle1 = muscle[0].properties;
                axis_model.muscle2 = muscle[1].properties;
                axis_model.limits  = std::make_pair(-M_PI / 2.0, M_PI / 2.0);
                utility::io::debug.out("OneAxis Initialisation\n");
            }

            OneAxis(std::vector<module::HardwareIO::muscle_t>& muscle, float mass, float radius, T controller)
                : muscle1(muscle[0]), muscle2(muscle[1]), controller(controller) {
                axis_model.mass    = mass;
                axis_model.radius  = radius;
                axis_model.P_a     = 101325;  // 1 atm
                axis_model.muscle1 = muscle[0].properties;
                axis_model.muscle2 = muscle[1].properties;
                axis_model.limits  = std::make_pair(-M_PI / 2.0, M_PI / 2.0);
                utility::io::debug.out("OneAxis Initialisation\n");
            }

            void Compute(float theta) {
                // float muscle1GetPosition = muscle1.GetPosition();
                // float muscle1GetVelocity = muscle1.GetVelocity();
                // float muscle1GetPressure = muscle1.GetPressure();
                // float muscle2GetPosition = muscle2.GetPosition();
                // float muscle2GetVelocity = muscle2.GetVelocity();
                // float muscle2GetPressure = muscle2.GetPressure();

                // Calculate the position derivative for velocity
                // Append states to a state vector in the order required
                // TODO Measure P_t
                axis_model.P_t = 413685;  // 60 PSI

                std::vector<float> states = {0.0, 0, 413685, 413685};
                //{muscle1.GetPosition(), muscle1.GetVelocity(), muscle1.GetPressure(), muscle2.GetPressure()};

                // Saturate the input to something sensible
                theta = utility::math::sat(theta, axis_model.limits);

                // // Call the MPC compute
                std::pair<bool, bool> valve_state = controller.Compute(axis_model, states, theta);

                muscle1.SetValveState(valve_state.first);
                muscle2.SetValveState(valve_state.second);
            }

            void UpdateVelocity() {
                muscle1.UpdateVelocity();
                muscle2.UpdateVelocity();
            }

        private:
            module::HardwareIO::Muscle muscle1;
            module::HardwareIO::Muscle muscle2;
            T controller;
            axis_model_t axis_model;
        };

    }  // namespace joint
}  // namespace HardwareIO
}  // namespace module

#endif  // MODULE_ONE_AXIS_HPP