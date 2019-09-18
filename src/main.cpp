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
#include "MPC/AdaptiveMPC/AdaptiveMPC.hpp"
#include "MPC/AdaptiveMPC/Optimizer.hpp"
#include "Sensors/LinearPotentiometer/LinearPotentiometer.hpp"
#include "Sensors/PressureSensor/PressureSensor.hpp"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "nuclear/src/clock.hpp"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_it.h"
#include "tim.h"
#include "usart.h"
#include "utility/io/adc.hpp"
#include "utility/io/uart.hpp"

extern "C" {
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();
    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 432;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Activate the Over-Drive mode
     */
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
    PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }
}
// void SystemClock_Config(void) {
//     RCC_OscInitTypeDef RCC_OscInitStruct;
//     RCC_ClkInitTypeDef RCC_ClkInitStruct;
//     RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

//     // Configure LSE Drive Capability
//     HAL_PWR_EnableBkUpAccess();

//     // Configure the main internal regulator output voltage
//     __HAL_RCC_PWR_CLK_ENABLE();
//     __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//     // Initializes the CPU, AHB and APB busses clocks
//     RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//     RCC_OscInitStruct.HSEState       = RCC_HSE_BYPASS;
//     RCC_OscInitStruct.HSIState       = RCC_HSE_OFF;
//     RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
//     RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
//     RCC_OscInitStruct.PLL.PLLM       = 8;
//     RCC_OscInitStruct.PLL.PLLN       = 432;
//     RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
//     RCC_OscInitStruct.PLL.PLLQ       = 9;
//     if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
//         Error_Handler();
//     }

//     // Activate the Over-Drive mode
//     if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
//         Error_Handler();
//     }

//     // Initializes the CPU, AHB and APB busses clocks
//     RCC_ClkInitStruct.ClockType =
//         (RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
//     RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
//     RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
//     RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//     RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

//     if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
//         Error_Handler();
//     }
//     PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
//     PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
//     if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
//         Error_Handler();
//     }
// }

void Error_Handler(void) {
    while (1) {
        utility::io::debug.out("ERROR\n");
    }
}
}

int main() {

    // Initialise the HAL driver
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART6_UART_Init();
    utility::clock::initialise();
    utility::io::adc_io.initialise();

    auto time_start = NUClear::clock::now();

    utility::io::debug.out("Welcome to PNEUbot\n");

    module::HardwareIO::muscle_properties_t pm_280 = {0.28, 0.33, 0.02};

    std::vector<module::HardwareIO::muscle_t> muscles;

    module::HardwareIO::muscle_t muscle1 = {
        module::HardwareIO::valve1, module::Sensors::pressuresensor1, module::Sensors::linearpot1, pm_280};

    muscles.push_back(muscle1);
    muscles.push_back(muscle1);

    module::HardwareIO::joint::OneAxis one_axis_muscle(muscles, 0.47, module::MPC::AdaptiveMPC::mpc);

    utility::io::debug.out("Initialisation Finished\n");

    utility::io::adc_io.Start();

    double Sampling_time = 10;  // 0.01 T_s

    while (1) {
        if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
            Error_Handler();
        }
        if (HAL_ADC_Start_IT(&hadc3) != HAL_OK) {
            Error_Handler();
        }

        auto now = NUClear::clock::now();
        HAL_Delay(500);
        double time = std::chrono::duration_cast<std::chrono::milliseconds>(now - time_start).count();

        utility::io::debug.out("PNEUBot is running %lf\n", time / 1000);

        // TODO
        // Going to need a Sampling time controller to handle the periodicity of the code
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - time_start).count() >= Sampling_time) {
            // Time to run our controller again

            one_axis_muscle.Compute(module::Sensors::linearpot2.GetPosition());
        }
    }
}

// TODO I think this is how the system needs to work
// Joints know about the muscles, a joint contains all of the muscles it needs to perform. Each muscle has knowledge of
// its respective sensors, pressure, position. A muscle is responsible for reading it's sensors. The joint compute
// function then calls the appropriate MPC function and arranges the inputs how the MPC expects. Something like this