#include "ADC.hpp"
#include "../../../nuclear/src/util/critical_section.hpp"
#include "adc.h"
#include "stm32f7xx.h"
#include "utility/io/uart.hpp"

uint16_t raw_sensors[1];

extern "C" {
__IO uint16_t uhADCxConvertedValue = 0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
    if (AdcHandle == &hadc1) {
        // uhADCxConvertedValue = HAL_ADC_GetValue(AdcHandle);
        // utility::io::debug.out("Length %d\n", uhADCxConvertedValue);
        utility::io::debug.out("I got an ADC interupt\n");
        utility::io::debug.out("Raw sensors value %d\n", raw_sensors[0]);
    }
}

// ADC1 init function
ADC_HandleTypeDef hadc1        = {0};
ADC_HandleTypeDef hadc3        = {0};
DMA_HandleTypeDef hdma_adc1    = {0};
DMA_HandleTypeDef hdma_adc3    = {0};
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
    hadc1.Init.NbrOfConversion       = 1;  // 12;
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
}

void MX_ADC3_Init(void) {

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    hadc3.Instance = ADC3;
    if (HAL_ADC_DeInit(&hadc3) != HAL_OK) {
        Error_Handler();
    }

    hadc3.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV8;
    hadc3.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode          = ADC_SCAN_ENABLE;
    hadc3.Init.ContinuousConvMode    = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion       = 1;  // 12;
    hadc3.Init.DMAContinuousRequests = DISABLE;
    hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel      = ADC_CHANNEL_10;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        Error_Handler();
    }
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

        /* ADC1 DMA Init */
        /* ADC1 Init */
        hdma_adc1.Instance                 = DMA2_Stream4;
        hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode                = DMA_NORMAL;
        hdma_adc1.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);

        // ADC1 interrupt Init
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }
    else {
        Error_Handler();
    }

    GPIO_InitStruct = {0};
    if (adcHandle->Instance == ADC3) {
        // ADC3 clock enable
        __HAL_RCC_ADC3_CLK_ENABLE();

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

        /* ADC3 DMA Init */
        /* ADC3 Init */
        hdma_adc3.Instance                 = DMA2_Stream1;
        hdma_adc3.Init.Channel             = DMA_CHANNEL_0;
        hdma_adc3.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc3.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc3.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc3.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc3.Init.Mode                = DMA_NORMAL;
        hdma_adc3.Init.Priority            = DMA_PRIORITY_LOW;
        hdma_adc3.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc3) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc3);

        // ADC3 interrupt Init
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

        HAL_DMA_DeInit(adcHandle->DMA_Handle);
    }
    if (adcHandle->Instance == ADC3) {
        // Peripheral clock disable
        __HAL_RCC_ADC3_CLK_DISABLE();

        // ADC3 GPIO Configuration
        // PA3     ------> ADC3_IN3
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

        // ADC3 interrupt Deinit
        HAL_NVIC_DisableIRQ(ADC_IRQn);

        HAL_DMA_DeInit(adcHandle->DMA_Handle);
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
        // if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
        //     utility::io::debug.out("ERROR: Could not initialise acd1\n");
        // }

        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &raw_sensors, sizeof(raw_sensors) / sizeof(uint16_t)) != HAL_OK) {
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