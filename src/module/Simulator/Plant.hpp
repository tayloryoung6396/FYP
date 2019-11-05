#ifndef MODULE_SIMULATED_PLANT_HPP
#define MODULE_SIMULATED_PLANT_HPP

#define _USE_MATH_DEFINES
#define M_PI 3.14159265358979323846

#include <math.h>
#include <Eigen/Core>
#include <algorithm>
#include <functional>
#include <numeric>
#include <vector>
#include "main.h"
#include "utility/io/uart.hpp"
#include "utility/math/saturation.hpp"

namespace module {
namespace Simulator {

    class Plant {
    public:
        Plant(std::vector<module::HardwareIO::muscle_t>& muscle, float mass, float radius);

        template <typename T>
        void ProcessModel(const T& model,
                          std::vector<float>& states,
                          std::vector<bool>& valve_states,
                          std::vector<float>& output_states) {

            Eigen::Matrix<float, 4, 4> A_mat;

            if (valve_states[0] && !valve_states[1]) {
                // Negative
                Linearise(model,
                          states,
                          -dotV_a(states[2], model.P_a, model.muscle1),
                          dotV_a(model.P_t, states[3], model.muscle2),
                          -dotV_a_P_m1(states[2], model.P_a, model.muscle1, mode),
                          dotV_a_P_m2(model.P_t, states[3], model.muscle2, mode),
                          A_mat);
            }
            else if (!valve_states[0] && valve_states[1]) {
                // Positive
                Linearise(model,
                          states,
                          dotV_a(model.P_t, states[2], model.muscle1),
                          -dotV_a(states[3], model.P_a, model.muscle2),
                          dotV_a_P_m1(model.P_t, states[2], model.muscle1, mode),
                          -dotV_a_P_m2(states[3], model.P_a, model.muscle2, mode),
                          A_mat);
            }
            else if (valve_states[0] && valve_states[1]) {
                // No move
                Linearise(model,
                          states,
                          dotV_a(model.P_t, states[2], model.muscle1),
                          dotV_a(model.P_t, states[3], model.muscle2),
                          dotV_a_P_m1(model.P_t, states[2], model.muscle1, mode),
                          dotV_a_P_m2(model.P_t, states[3], model.muscle2, mode),
                          A_mat);
            }
            else {
                Error_Handler();
            }

            float Sampling_time2 = 0.05;  // 0.01 T_s

            Eigen::Matrix<float, 4, 1> x_state(input_states[0], input_states[1], input_states[2], input_states[3]);

            Eigen::Matrix<float, 4, 1> x_states_update =
                (Eigen::Matrix<float, 4, 4>::Identity() + Sampling_time2 * A_mat) * x_state;

            output_states.push_back(utility::math::sat(x_states_update(0, 0), std::make_pair(M_PI / 2.0, -M_PI / 2.0)));
            output_states.push_back(utility::math::sat(x_states_update(1, 0), std::make_pair(10, -10)));
            output_states.push_back(utility::math::sat(x_states_update(2, 0), std::make_pair(413685, 0)));
            output_states.push_back(utility::math::sat(x_states_update(3, 0), std::make_pair(413685, 0)));

            return;
        }

        template <typename T>
        float dotV_a(float P_1, float P_2, const T& m) {
            float b       = m.critical_ratio;
            float C       = m.sonic_conductance;
            float A       = C * std::sqrt(m.T_0 / m.T_1);
            float P_b     = P_2 / P_1 - b;
            float b_1     = 1 - b;
            float P_bsqrt = std::sqrt(1 - (P_b / b_1) * (P_b / b_1));

            if (P_2 / P_1 > b) {
                return std::isinf(P_1 * A * P_bsqrt) ? 0 : P_1 * A * P_bsqrt;
            }
            return std::isinf(P_1 * A) ? 0 : P_1 * A;
        }

        template <typename T>
        float dotV_a_P_m1(float P_1, float P_2, const T& m, int mode) {
            float b       = m.critical_ratio;
            float C       = m.sonic_conductance;
            float A       = C * std::sqrt(m.T_0 / m.T_1);
            float P_b     = P_2 / P_1 - b;
            float i       = A * P_b;
            float b_1     = 1 - b;
            float P_bsqrt = std::sqrt(1 - (P_b / b_1) * (P_b / b_1));

            if (mode == 2 || mode == 3) {
                // P_2 == P_m1
                if (P_2 / P_1 > b) {
                    return std::isinf(-(i / (b_1 * b_1 * P_bsqrt))) ? 0 : -(i / (b_1 * b_1 * P_bsqrt));
                }
                return 0;
            }
            else {
                // P_1 == P_m1
                if (P_2 / P_1 > b) {
                    return std::isinf(A * P_bsqrt + (i * P_2 / (b_1 * b_1 * P_1 * P_bsqrt)))
                               ? 0
                               : A * P_bsqrt + (i * P_2 / (b_1 * b_1 * P_1 * P_bsqrt));
                }

                return std::isinf(C * std::sqrt(m.T_0 / m.T_1)) ? 0 : C * std::sqrt(m.T_0 / m.T_1);
            }
            return 0;
        }

        template <typename T>
        float dotV_a_P_m2(float P_1, float P_2, const T& m, int mode) {
            float b       = m.critical_ratio;
            float C       = m.sonic_conductance;
            float A       = C * std::sqrt(m.T_0 / m.T_1);
            float P_b     = P_2 / P_1 - b;
            float i       = A * P_b;
            float b_1     = 1 - b;
            float P_bsqrt = std::sqrt(1 - (P_b / b_1) * (P_b / b_1));

            if (mode == 1 || mode == 3) {
                // P_2 == P_m2
                if (P_2 / P_1 > b) {
                    // utility::io::debug.out("dotV_a_P_m2 mode 13 >\n");
                    return std::isinf(-i / (b_1 * b_1 * P_bsqrt)) ? 0 : -i / (b_1 * b_1 * P_bsqrt);
                }
                // utility::io::debug.out("dotV_a_P_m2 mode 13 <\n");
                return 0;
            }
            else {
                // P_1 == P_m2
                if (P_2 / P_1 > b) {
                    // utility::io::debug.out("dotV_a_P_m2 mode 2 >\n");
                    return std::isinf(A * P_bsqrt + ((P_2 * i) / (b_1 * b_1 * P_1 * P_bsqrt)))
                               ? 0
                               : A * P_bsqrt + ((P_2 * i) / (b_1 * b_1 * P_1 * P_bsqrt));
                }

                // utility::io::debug.out("dotV_a_P_m2 mode 2 <\n");
                return std::isinf(C * std::sqrt(m.T_0 / m.T_1)) ? 0 : C * std::sqrt(m.T_0 / m.T_1);
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
                       Eigen::Matrix<float, 4, 4>& A_mat) {

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

				float Lin_mat_1_0 = (-F_s2 - F_s1) / mass;

				/********************************************* Lin_mat_2_0 ********************************************/
				dV_m1          = (3 * a1 * k1 * k1 + 2 * b1 * k1 + c1) / L_10;
				ddotV_m1_dV_m1 =
					((a1 * k1 * k1 * k1 + b1 * k1 * k1 + c1 * k1 + d1) 
						* (6 * a1 * k1 * dotk1 + 2 * b1 * dotk1)
					- (3 * a1 * k1 * k1 * dotk1 + 2 * b1 * k1 * dotk1 + c1 * dotk1) 
						* (3 * a1 * k1 * k1 * dotk1 + 2 * b1 * k1 * dotk1 + c1 * dotk1))
					/ (L_10 * (a1 * k1 * k1 * k1 + b1 * k1 * k1 + c1 * k1 + d1)
					   * (a1 * k1 * k1 * k1 + b1 * k1 * k1 + c1 * k1 + d1));

				float Lin_mat_2_0 = (P_a * dotV_a1 / dV_m1) - (P_m1 * ddotV_m1_dV_m1);
				/********************************************* Lin_mat_3_0 ********************************************/
				dV_m2          = (3 * a2 * k2 * k2 + 2 * b2 * k2 + c2) / L_20;
				ddotV_m2_dV_m2 =
					((a2 * k2 * k2 * k2 + b2 * k2 * k2 + c2 * k2 + d2) 
						* (6 * a2 * k2 * dotk2 + 2 * b2 * dotk2)
					- (3 * a2 * k2 * k2 * dotk2 + 2 * b2 * k2 * dotk2 + c2 * dotk2) 
						* (3 * a2 * k2 * k2 * dotk2 + 2 * b2 * k2 * dotk2 + c2 * dotk2))
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

				float Lin_mat_1_1 = (-F_d2 - F_d1) / mass;
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
				
				// TODO This is incorrect, but i don't know any better
				float Lin_mat_1_2 = (F_s1 + F_d1) / mass;
				/********************************************* Lin_mat_2_2 ********************************************/
				dotV_m1 = (3 * a1 * k1 * k1 * dotk1 + 2 * b1 * k1 * dotk1 + c1 * dotk1) ;
				V_m1    = (a1 * k1 * k1 * k1) + (b1 * k1 * k1) + (c1 * k1) + d1;

				float Lin_mat_2_2 = (P_a * ddotV_a1 - dotV_m1) / V_m1 + 0.34;

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
				
				float Lin_mat_1_3 = (-F_s2 - F_d2) / mass;
				/********************************************* Lin_mat_2_3 ********************************************/
				float Lin_mat_2_3 = 0;
				/********************************************* Lin_mat_3_3 ********************************************/
				dotV_m2 = (3 * a2 * k2 * k2 * dotk2 + 2 * b2 * k2 * dotk2 + c2 * dotk2) ;
				V_m2    = (a2 * k2 * k2 * k2) + (b2 * k2 * k2) + (c2 * k2) + d2;
				
				float Lin_mat_3_3 = (P_a * ddotV_a2 - dotV_m2) / V_m2 + 0.34;

				A_mat << Lin_mat_0_0, Lin_mat_0_1, Lin_mat_0_2, Lin_mat_0_3, 
						 Lin_mat_1_0, Lin_mat_1_1, Lin_mat_1_2, Lin_mat_1_3, 
						 Lin_mat_2_0, Lin_mat_2_1, Lin_mat_2_2, Lin_mat_2_3, 
						 Lin_mat_3_0, Lin_mat_3_1, Lin_mat_3_2, Lin_mat_3_3;
            /* clang-format on */
        }

        void Compute();

    private:
        module::HardwareIO::Muscle muscle1;
        module::HardwareIO::Muscle muscle2;
        axis_model_t axis_model;
    };

    extern Plant plant;
}  // namespace Simulator
}  // namespace module

#endif  // MODULE_SIMULATED_PLANT_HPP