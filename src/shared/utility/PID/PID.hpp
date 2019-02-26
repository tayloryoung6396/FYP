#ifndef MODULE_PID_HPP
#define MODULE_PID_HPP

namespace shared {
namespace utility {
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
}  // namespace utility
}  // namespace shared

#endif  // MODULE_PID_HPP


// #ifndef _PID_H_
// #define _PID_H_

// class PIDImpl;
// class PID
// {
//     public:
//         // Kp -  proportional gain
//         // Ki -  Integral gain
//         // Kd -  derivative gain
//         // dt -  loop interval time
//         // max - maximum value of manipulated variable
//         // min - minimum value of manipulated variable
//         PID( double dt, double max, double min, double Kp, double Kd, double Ki );

//         // Returns the manipulated variable given a setpoint and current process value
//         double calculate( double setpoint, double pv );
//         ~PID();

//     private:
//         PIDImpl *pimpl;
// };

// #endif