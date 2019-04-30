#include "ADC.hpp"
#include "../../../nuclear/src/util/critical_section.hpp"
#include "adc.h"
#include "stm32f7xx.h"

extern "C" {
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc == &hadc1) {
        shared::utility::adc_io.FillSensors(&hadc1, 0);
    }
    else if (hadc == &hadc3) {
        shared::utility::adc_io.FillSensors(&hadc3, 12);
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {}
}

namespace shared {
namespace utility {

    ADC_IO adc_io = ADC_IO();

    ADC_IO::ADC_IO() {
        HAL_ADC_Start_IT(&hadc1);
        HAL_ADC_Start_IT(&hadc3);
    }

    void ADC_IO::FillSensors(ADC_HandleTypeDef* hadc, int offset) {
        NUClear::util::critical_section lock;
        for (int i = offset; i < offset + 12; i++) {
            raw_data.sensors[i] = HAL_ADC_GetValue(hadc);
        }
        lock.release();
    }

    uint16_t ADC_IO::GetSensors(int port) {

        NUClear::util::critical_section lock;
        uint16_t data = raw_data.sensors[port];
        lock.release();
        return (data);
    }

}  // namespace utility
}  // namespace shared