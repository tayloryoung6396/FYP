#ifndef MODULE_MUSCLE_HPP
#define MODULE_MUSCLE_HPP

#include <vector>
#include "HardwareIO/Valve/Valve.hpp"
#include "Sensors/LinearPotentiometer/LinearPotentiometer.hpp"
#include "Sensors/PressureSensor/PressureSensor.hpp"

namespace module {
namespace HardwareIO {

    struct muscle_properties_t {
        float L_0;
        float K_0;
        float critical_ratio;
        float sonic_conductance;
        float T_0;
        float T_1;
        float damping_coefficient;
        float muscle_coefficients[4];
        float F_ce[6][6];
    };

    struct muscle_t {
        Valve& valve;
        Sensors::PressureSensor& pressure_sensor;
        Sensors::LinearPot& linear_pot;
        muscle_properties_t properties;
    };

    class Muscle {
    public:
        Muscle(module::HardwareIO::muscle_t muscle);


        float GetPosition();
        void UpdateVelocity();
        float GetVelocity();
        float GetPressure();
        void SetValveState(bool state);
        bool GetValveState();

    private:
        std::vector<float> prev_position;

        Valve& valve;
        Sensors::PressureSensor& pressure_sensor;
        Sensors::LinearPot& linear_pot;
        muscle_properties_t properties;
    };
}  // namespace HardwareIO
}  // namespace module

#endif  // MODULE_MUSCLE_HPP