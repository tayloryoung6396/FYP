#include "ADC.hpp"
#include "../../../nuclear/src/util/critical_section.hpp"
#include "adc.h"
#include "stm32f7xx.h"
#include "utility/io/uart.hpp"

extern "C" {
__IO uint16_t uhADCxConvertedValue  = 0;
__IO uint16_t uhADCxConvertedValue2 = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
    if (AdcHandle == &hadc1) {
        uhADCxConvertedValue = HAL_ADC_GetValue(AdcHandle);


        uhADCxConvertedValue2 = HAL_ADC_GetValue(AdcHandle);
        utility::io::adc_io.FillSensors2(uhADCxConvertedValue, 0);
        utility::io::adc_io.FillSensors2(uhADCxConvertedValue2, 1);

        // utility::io::debug.out("Got ADC 1 %d\n", uhADCxConvertedValue);
        // utility::io::debug.out("Got ADC 2 %d\n", uhADCxConvertedValue2);
        // utility::io::adc_io.FillSensors(&hadc1, 0);
    }
}

// ADC1 init function
ADC_HandleTypeDef hadc1        = {0};
ADC_ChannelConfTypeDef sConfig = {0};

void MX_ADC1_Init(void) {

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    hadc1.Instance = ADC1;
    if (HAL_ADC_DeInit(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 2;  // 12;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel      = ADC_CHANNEL_10;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank    = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    // sConfig.Channel = ADC_CHANNEL_3;
    // sConfig.Rank    = 3;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    // Error_Handler();
    // }
    // sConfig.Channel = ADC_CHANNEL_3;
    // sConfig.Rank    = 4;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    // Error_Handler();
    // }
    // sConfig.Channel = ADC_CHANNEL_3;
    // sConfig.Rank    = 5;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    // Error_Handler();
    // }
    // sConfig.Channel = ADC_CHANNEL_3;
    // sConfig.Rank    = 6;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    // Error_Handler();
    // }
    // sConfig.Channel = ADC_CHANNEL_3;
    // sConfig.Rank    = 7;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    // Error_Handler();
    // }
    // sConfig.Channel = ADC_CHANNEL_3;
    // sConfig.Rank    = 8;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    // Error_Handler();
    // }
    // sConfig.Channel = ADC_CHANNEL_3;
    // sConfig.Rank    = 9;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    // Error_Handler();
    // }
    // sConfig.Channel = ADC_CHANNEL_3;
    // sConfig.Rank    = 10;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    // Error_Handler();
    // }
    // sConfig.Channel = ADC_CHANNEL_3;
    // sConfig.Rank    = 11;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    // Error_Handler();
    // }
    // sConfig.Channel = ADC_CHANNEL_3;
    // sConfig.Rank    = 12;
    // if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    // Error_Handler();
    // }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (adcHandle->Instance == ADC1) {
        // ADC1 clock enable
        __HAL_RCC_ADC1_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin  = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // ADC1 interrupt Init
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }
    else {
        Error_Handler();
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle) {

    if (adcHandle->Instance == ADC1) {
        // Peripheral clock disable
        __HAL_RCC_ADC1_CLK_DISABLE();

        // ADC1 GPIO Configuration
        // PA3     ------> ADC1_IN3
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

        // ADC1 interrupt Deinit
        HAL_NVIC_DisableIRQ(ADC_IRQn);
    }
    else {
        Error_Handler();
    }
}
}

namespace utility {
namespace io {

    ADC_IO adc_io = ADC_IO();

    ADC_IO::ADC_IO() {}

    void ADC_IO::initialise() { MX_ADC1_Init(); }

    void ADC_IO::Start() {
        if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
            utility::io::debug.out("ERROR: Could not initialise acd1\n");
        }
    }

    void ADC_IO::FillSensors(ADC_HandleTypeDef* hadc, int offset) {
        // NUClear::util::critical_section lock;
        // for (int i = offset; i < offset + 2;
        //      i++) {  // TODO This should be increased to the number of channels to be read in
        //     raw_data.sensors[i] = HAL_ADC_GetValue(hadc);
        //     // if (raw_data.sensors[i] != 0) {
        //     utility::io::debug.out("Data found! Channel [%d] : %d\n", i, raw_data.sensors[i]);
        //     // }
        // }
        // lock.release();
    }

    void ADC_IO::FillSensors2(uint16_t value, int offset) {
        // raw_data.sensors[offset] = value;
        // utility::io::debug.out("Data found! Channel [%d] : %d\n", offset, raw_data.sensors[offset]);
        utility::io::debug.out("Data found! %d\n", value);
    }

    uint16_t ADC_IO::GetSensors(int port) {
        NUClear::util::critical_section lock;
        uint16_t data = raw_data.sensors[port];
        utility::io::debug.out("Getting sensor value: %d\n", data);
        lock.release();
        return (data);
    }

}  // namespace io
}  // namespace utility