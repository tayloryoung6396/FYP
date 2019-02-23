#ifndef MODULE_PID_HPP
#define MODULE_PID_HPP

namespace module {
class PID {
public:
    PID();  // PressureSensor& pressure_sensor, LinearPot& linear_pot);

    void Compute();

private:
    double proportional;
    double intergral;
    double differential;
};
}  // namespace module

#endif  // MODULE_PID_HPP