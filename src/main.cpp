#include "main.hpp"
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include "HardwareIO/Joint/LinearAxis/LinearAxis.hpp"
#include "HardwareIO/Joint/OneAxis/OneAxis.hpp"
#include "HardwareIO/Joint/TwoAxis/TwoAxis.hpp"
#include "HardwareIO/Muscle/Muscle.hpp"
#include "HardwareIO/Valve/Valve.hpp"
#include "Input/Controller/Controller.hpp"
#include "Sensors/LinearPotentiometer/LinearPotentiometer.hpp"
#include "Sensors/PressureSensor/PressureSensor.hpp"
#include "adc.h"
#include "gpio.h"
// #include "i2c.h"
#include "main.h"
#include "nuclear/src/clock.hpp"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_it.h"
#include "tim.h"
#include "usart.h"
// #include "usb_otg.h"
#include "utility/PID/PID.hpp"
#include "utility/io/adc.hpp"
#include "utility/io/uart.hpp"

extern "C" {
void SystemClock_Config(void);
}

int main() {

    // Initialise the HAL driver
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();
    utility::clock::initialise();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC3_Init();
    // MX_I2C1_Init();
    // MX_USART2_UART_Init();
    MX_USART6_UART_Init();
    // MX_USB_OTG_FS_PCD_Init();

    auto time_start = NUClear::clock::now();

    utility::io::debug.out("Welcome to PNEUbot\n");

    std::vector<module::HardwareIO::muscle_t> muscles;

    module::HardwareIO::muscle_t muscle1 = {module::HardwareIO::valve1,
                                            module::Sensors::pressuresensor1,
                                            module::Sensors::linearpot1,
                                            shared::utility::pid1,
                                            0.1};

    muscles.push_back(muscle1);

    module::HardwareIO::joint::LinearAxis linear_muscle(muscles);

    int i = 0;

    while (1) {
        i++;

        HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
        HAL_Delay(300);
        auto now    = NUClear::clock::now();
        double time = std::chrono::duration_cast<std::chrono::microseconds>(now - time_start).count();
        utility::io::debug.out("PNEUBot is running %lf\n", time);

        // This should probably be handled by ome controller
        // Set the value to the position requested
        linear_muscle.Compute(0.1);
    }
}

// RCC_OscInitTypeDef RCC_OscInitStruct         = {0};
// RCC_ClkInitTypeDef RCC_ClkInitStruct         = {0};
// RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

// /**Configure LSE Drive Capability
//  */
// HAL_PWR_EnableBkUpAccess();
// /**Configure the main internal regulator output voltage
//  */
// __HAL_RCC_PWR_CLK_ENABLE();
// __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
// /**Initializes the CPU, AHB and APB busses clocks
//  */
// RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
// RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
// RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
// RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
// if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//     Error_Handler();
// }
// /**Initializes the CPU, AHB and APB busses clocks
//  */
// RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
// RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
// RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
// RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
// RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

// if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
//     Error_Handler();
// }
// PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
// PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
// if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
//     Error_Handler();
// }