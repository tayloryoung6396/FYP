#ifndef MODULE_PID_HPP
#define MODULE_PID_HPP

namespace module {
class PID {
public:
    PID();  // PressureSensor& pressure_sensor, LinearPot& linear_pot);

    void Compute();

    // operator bool();
    // bool operator=(const bool& value);
    // bool operator!();


private:
    double proportional;
    double intergral;
    double differential;
};

extern PID pid1;
extern PID pid2;
}  // namespace module

#endif  // MODULE_PID_HPP