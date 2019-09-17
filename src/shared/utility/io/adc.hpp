#ifndef UTILITY_ADC_HPP
#define UTILITY_ADC_HPP

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
        uint16_t GetSensors(int port);
        // void FillSensors(ADC_HandleTypeDef* hadc, int offset);
        // void FillSensors2(uint16_t value, int offset);

    private:
        ADC_Sensors raw_data;
    };

    extern ADC_IO adc_io;

}  // namespace io
}  // namespace utility

#endif  // UTILITY_ADC_HPP