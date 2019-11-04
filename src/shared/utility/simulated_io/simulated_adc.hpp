#ifndef UTILITY_SIMULATED_ADC_HPP
#define UTILITY_SIMULATED_ADC_HPP

#include <stdint.h>
#include "stm32f7xx.h"

namespace utility {
namespace io {
    struct ADC_Sensors {
        uint16_t sensors[9];
    };

    class ADC_IO {
    public:
        ADC_IO();

        void initialise();
        void Start();
        void StartADC1();
        void StartADC3();
        uint16_t GetSensors(int port);
        void PrintSensors();

    private:
        ADC_Sensors raw_data;
        ADC_Sensors safe_data;
    };

    extern ADC_IO adc_io;

}  // namespace io
}  // namespace utility

#endif  // UTILITY_SIMULATED_ADC_HPP