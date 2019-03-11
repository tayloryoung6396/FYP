#include "ADC.hpp"

namespace shared {
namespace utility {

    ADC::ADC() {

        ADC_ChannelConfTypeDef sConfig = {0};
        /**Configure the global features of the ADC (Clock, Resolution, Data Alignment
         * and number of conversion)
         */
        hadc1.Instance                   = ADC1;
        hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
        hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
        hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
        hadc1.Init.ContinuousConvMode    = DISABLE;
        hadc1.Init.DiscontinuousConvMode = DISABLE;
        hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
        hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
        hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
        hadc1.Init.NbrOfConversion       = 12;
        hadc1.Init.DMAContinuousRequests = DISABLE;
        hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
        if (HAL_ADC_Init(&hadc1) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel      = ADC_CHANNEL_0;
        sConfig.Rank         = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_1;
        sConfig.Rank    = ADC_REGULAR_RANK_2;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_2;
        sConfig.Rank    = ADC_REGULAR_RANK_3;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_3;
        sConfig.Rank    = ADC_REGULAR_RANK_4;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_4;
        sConfig.Rank    = ADC_REGULAR_RANK_5;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_5;
        sConfig.Rank    = ADC_REGULAR_RANK_6;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_6;
        sConfig.Rank    = ADC_REGULAR_RANK_7;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_7;
        sConfig.Rank    = ADC_REGULAR_RANK_8;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_8;
        sConfig.Rank    = ADC_REGULAR_RANK_9;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_9;
        sConfig.Rank    = ADC_REGULAR_RANK_10;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_14;
        sConfig.Rank    = ADC_REGULAR_RANK_11;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_15;
        sConfig.Rank    = ADC_REGULAR_RANK_12;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
            Error_Handler();
        }


        ADC_ChannelConfTypeDef sConfig = {0};
        /**Configure the global features of the ADC (Clock, Resolution, Data Alignment
         * and number of conversion)
         */
        hadc3.Instance                   = ADC3;
        hadc3.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
        hadc3.Init.Resolution            = ADC_RESOLUTION_12B;
        hadc3.Init.ScanConvMode          = ADC_SCAN_ENABLE;
        hadc3.Init.ContinuousConvMode    = DISABLE;
        hadc3.Init.DiscontinuousConvMode = DISABLE;
        hadc3.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
        hadc3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
        hadc3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
        hadc3.Init.NbrOfConversion       = 12;
        hadc3.Init.DMAContinuousRequests = DISABLE;
        hadc3.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
        if (HAL_ADC_Init(&hadc3) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel      = ADC_CHANNEL_4;
        sConfig.Rank         = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_5;
        sConfig.Rank    = ADC_REGULAR_RANK_2;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_6;
        sConfig.Rank    = ADC_REGULAR_RANK_3;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_7;
        sConfig.Rank    = ADC_REGULAR_RANK_4;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_8;
        sConfig.Rank    = ADC_REGULAR_RANK_5;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_9;
        sConfig.Rank    = ADC_REGULAR_RANK_6;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_10;
        sConfig.Rank    = ADC_REGULAR_RANK_7;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_11;
        sConfig.Rank    = ADC_REGULAR_RANK_8;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_12;
        sConfig.Rank    = ADC_REGULAR_RANK_9;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_13;
        sConfig.Rank    = ADC_REGULAR_RANK_10;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_14;
        sConfig.Rank    = ADC_REGULAR_RANK_11;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
        sConfig.Channel = ADC_CHANNEL_15;
        sConfig.Rank    = ADC_REGULAR_RANK_12;
        if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
            Error_Handler();
        }
    }

}  // namespace utility
}  // namespace shared