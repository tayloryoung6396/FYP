#ifndef UTILITY_ADC_HPP
#define UTILITY_ADC_HPP

#include <stdint.h>
#include "stm32f7xx.h"

namespace shared {
namespace utility {
    struct ADC_Sensors {
        uint16_t sensors[12];
    };

    class ADC_IO {
    public:
        ADC_IO();
        uint16_t GetSensors(int port);
        void FillSensors(ADC_HandleTypeDef* hadc, int offset);

    private:
        ADC_Sensors raw_data;
    };

    extern ADC_IO adc_io;

}  // namespace utility
}  // namespace shared

#endif  // UTILITY_ADC_HPP