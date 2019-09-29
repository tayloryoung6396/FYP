#include "ADC.hpp"
#include "../../../nuclear/src/util/critical_section.hpp"
#include "adc.h"
#include "stm32f7xx.h"
#include "utility/io/uart.hpp"

extern "C" {
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle) {
    if (AdcHandle == &hadc1) {
        // TODO Some sort of critical section should be used for these maybe?
        utility::io::debug.out("ADC1\n");
    }
    else if (AdcHandle == &hadc3) {
        // TODO Some sort of critical section should be used for these maybe?
        utility::io::debug.out("ADC3\n");
    }
    else {
        utility::io::debug.out("I got an interupt but i dont know why\n");
    }
}

// ADC1 init function
ADC_HandleTypeDef hadc1        = {0};
ADC_HandleTypeDef hadc3        = {0};
DMA_HandleTypeDef hdma_adc1    = {0};
DMA_HandleTypeDef hdma_adc3    = {0};
ADC_ChannelConfTypeDef sConfig = {0};
void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 4;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel      = ADC_CHANNEL_0;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank    = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank    = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank    = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}
// ADC3 init function
void MX_ADC3_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    hadc3.Instance                   = ADC3;
    hadc3.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode          = ENABLE;
    hadc3.Init.ContinuousConvMode    = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion       = 5;
    hadc3.Init.DMAContinuousRequests = ENABLE;
    hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc3) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel      = ADC_CHANNEL_4;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank    = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel = ADC_CHANNEL_12;
    sConfig.Rank    = ADC_REGULAR_RANK_3;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel = ADC_CHANNEL_13;
    sConfig.Rank    = ADC_REGULAR_RANK_4;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    sConfig.Channel = ADC_CHANNEL_11;
    sConfig.Rank    = ADC_REGULAR_RANK_5;
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (adcHandle->Instance == ADC1) {


        // ADC1 clock enable
        __HAL_RCC_ADC1_CLK_ENABLE();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        // ADC1 GPIO Configuration
        // PA0-WKUP     ------> ADC1_IN0
        // PA1     ------> ADC1_IN1
        // PA2     ------> ADC1_IN2
        // PA3     ------> ADC1_IN3
        // PA4     ------> ADC1_IN4
        // PA5     ------> ADC1_IN5
        // PA6     ------> ADC1_IN6
        // PA7     ------> ADC1_IN7
        // PC4     ------> ADC1_IN14
        // PC5     ------> ADC1_IN15
        // PB0     ------> ADC1_IN8
        // PB1     ------> ADC1_IN9

        GPIO_InitStruct.Pin =
            GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // ADC1 DMA Init
        // ADC1 Init
        hdma_adc1.Instance                 = DMA2_Stream4;
        hdma_adc1.Init.Channel             = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode                = DMA_NORMAL;
        hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_adc1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc1);

        // ADC1 interrupt Init
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }
    else if (adcHandle->Instance == ADC3) {


        // ADC3 clock enable
        __HAL_RCC_ADC3_CLK_ENABLE();

        __HAL_RCC_GPIOF_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        // ADC3 GPIO Configuration
        // PF3     ------> ADC3_IN9
        // PF4     ------> ADC3_IN14
        // PF5     ------> ADC3_IN15
        // PF6     ------> ADC3_IN4
        // PF7     ------> ADC3_IN5
        // PF8     ------> ADC3_IN6
        // PF9     ------> ADC3_IN7
        // PF10     ------> ADC3_IN8
        // PC0     ------> ADC3_IN10
        // PC1     ------> ADC3_IN11
        // PC2     ------> ADC3_IN12
        // PC3     ------> ADC3_IN13
        // PA3     ------> ADC3_IN3

        GPIO_InitStruct.Pin =
            GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // ADC3 DMA Init
        // ADC3 Init
        hdma_adc3.Instance                 = DMA2_Stream1;
        hdma_adc3.Init.Channel             = DMA_CHANNEL_2;
        hdma_adc3.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_adc3.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_adc3.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc3.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
        hdma_adc3.Init.Mode                = DMA_NORMAL;
        hdma_adc3.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_adc3.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc3) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(adcHandle, DMA_Handle, hdma_adc3);

        // ADC3 interrupt Init
        HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle) {

    if (adcHandle->Instance == ADC1) {


        // Peripheral clock disable
        __HAL_RCC_ADC1_CLK_DISABLE();

        HAL_GPIO_DeInit(
            GPIOA,
            GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4 | GPIO_PIN_5);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1);

        // ADC1 DMA DeInit
        HAL_DMA_DeInit(adcHandle->DMA_Handle);

        // ADC1 interrupt Deinit

        // * Uncomment the line below to disable the "ADC_IRQn" interrupt
        // * Be aware, disabling shared interrupt may affect other IPs

        // HAL_NVIC_DisableIRQ(ADC_IRQn);
    }
    else if (adcHandle->Instance == ADC3) {


        // Peripheral clock disable
        __HAL_RCC_ADC3_CLK_DISABLE();

        HAL_GPIO_DeInit(
            GPIOF,
            GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);

        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

        // ADC3 DMA DeInit
        HAL_DMA_DeInit(adcHandle->DMA_Handle);

        // ADC3 interrupt Deinit

        // * Uncomment the line below to disable the "ADC_IRQn" interrupt
        // * Be aware, disabling shared interrupt may affect other IPs

        // HAL_NVIC_DisableIRQ(ADC_IRQn);
    }
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
        // NUClear::util::critical_section lock;
        if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &raw_data.sensors[0], 4) != HAL_OK) {
            utility::io::debug.out("ERROR: Could not start acd1 dma\n");
        }
        else {
            utility::io::debug.out("Started acd1 dma\n");
        }

        // if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &raw_data.sensors[4], 5) != HAL_OK) {
        //     utility::io::debug.out("ERROR: Could not start acd1 dma\n");
        // }
        // lock.release();
    }

    // void ADC_IO::StartADC1() {
    //     if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &raw_data.sensors[0], 4) != HAL_OK) {
    //         utility::io::debug.out("ERROR: Could not start acd1 dma\n");
    //     }
    //     else {
    //         utility::io::debug.out("Started acd1 dma\n");
    //     }
    // }

    // void ADC_IO::StartADC3() {
    //     if (HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &raw_data.sensors[4], 5) != HAL_OK) {
    //         utility::io::debug.out("ERROR: Could not start acd1 dma\n");
    //     }
    //     else {
    //         utility::io::debug.out("Started acd1 dma\n");
    //     }
    // }

    void ADC_IO::PrintSensors() {
        utility::io::debug.out("Raw sensors value\n%d\t%d\t%d\t%d\n%d\t%d\t%d\t%d\t%d\n",
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