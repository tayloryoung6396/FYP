#include "ADC.hpp"
#include "../../../nuclear/src/util/critical_section.hpp"
#include "adc.h"
#include "stm32f7xx.h"
#include "utility/io/uart.hpp"

static bool ADC_interrupt_1 = true;
static bool ADC_interrupt_3 = true;

extern "C" {
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
    if (AdcHandle == &hadc1) {
        ADC_interrupt_1 = true;
    }
    if (AdcHandle == &hadc3) {
        ADC_interrupt_3 = true;
    }
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc) {
    if (hadc == &hadc1) {
        utility::io::debug.out("ADC Error hadc1\n");
    }
    if (hadc == &hadc3) {
        utility::io::debug.out("ADC Error hadc3\n");
    }

    Error_Handler();

    // TODO Process here shoud be to reinit the relative functions and try again
}
}

namespace utility {
namespace io {

    ADC_IO adc_io = ADC_IO();

    ADC_IO::ADC_IO() {}

    void ADC_IO::initialise() {
        MX_ADC1_Init();
        MX_ADC3_Init();
        utility::io::debug.out("initialise ADC\n");
    }

    void ADC_IO::Start() {
        if (ADC_interrupt_1) {
            if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &(raw_data.sensors[0]), 4) != HAL_OK) {
                utility::io::debug.out("ERROR: Could not start acd1 dma\n");
                Error_Handler();
            }
            ADC_interrupt_1 = false;
        }
        if (ADC_interrupt_3) {
            if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &(raw_data.sensors[4]), 5) != HAL_OK) {
                utility::io::debug.out("ERROR: Could not start acd1 dma\n");
                Error_Handler();
            }
            ADC_interrupt_3 = false;
        }
    }

    void ADC_IO::StartADC1() {
        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &(raw_data.sensors[0]), 4) != HAL_OK) {
            utility::io::debug.out("ERROR: Could not start acd1 dma\n");
            Error_Handler();
        }
    }

    void ADC_IO::StartADC3() {
        if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &(raw_data.sensors[4]), 5) != HAL_OK) {
            utility::io::debug.out("ERROR: Could not start acd3 dma\n");
            Error_Handler();
        }
    }

    void ADC_IO::PrintSensors() {
        utility::io::debug.out("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
                               raw_data.sensors[0],
                               raw_data.sensors[1],
                               raw_data.sensors[2],
                               raw_data.sensors[3],
                               raw_data.sensors[4],
                               raw_data.sensors[5],
                               raw_data.sensors[6],
                               raw_data.sensors[7],
                               raw_data.sensors[8]);
    }

    uint16_t ADC_IO::GetSensors(int port) {
        NUClear::util::critical_section lock;
        uint16_t data = raw_data.sensors[port];
        lock.release();
        return (data);
    }

}  // namespace io
}  // namespace utility