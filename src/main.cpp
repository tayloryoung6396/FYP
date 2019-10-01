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
#include "utility/io/gpio.hpp"
#include "utility/io/uart.hpp"

extern "C" {
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    // Configure LSE Drive Capability
    HAL_PWR_EnableBkUpAccess();

    // Configure the main internal regulator output voltage
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Initializes the CPU, AHB and APB busses clocks
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

    // Activate the Over-Drive mode
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        Error_Handler();
    }

    // Initializes the CPU, AHB and APB busses clocks
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

void Error_Handler(void) {
    utility::io::debug.out("ERROR\n");
    utility::io::gpio::led3 = true;
    while (1) {
    }
}
}

int main() {
    /*******************************************************************************************************************
    ********************************************** System Initialisation ***********************************************
    *******************************************************************************************************************/
    // Initialise the HAL driver
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART6_UART_Init();
    utility::clock::initialise();
    utility::io::adc_io.initialise();

    /*******************************************************************************************************************
    ************************************************* Muscles & Joints *************************************************
    *******************************************************************************************************************/
    auto time_start = NUClear::clock::now();

    utility::io::debug.out("Welcome to PNEUbot\n");

    // We need our setpoint potentiometer
    auto Setpoint = module::Input::controller.GetPosition();


    // Declare our muscle parameters and populate a vector of muscles
    // float nom_length;
    // float contraction_percent;
    // float critical_ratio;
    // float sonic_conductance;
    // float T_0;
    // float T_1;
    // float damping_coefficient;
    // float muscle_coefficients[4];
    // float F_ce[6][6];

    module::HardwareIO::muscle_properties_t pm_280 = {0.28,
                                                      0.33,
                                                      0.433,
                                                      2.6167 * std::pow(10, 9),
                                                      25,
                                                      25,
                                                      1,
                                                      {1, 1, 1, 1},
                                                      {{1, 1, 1, 1, 1, 1},
                                                       {1, 1, 1, 1, 1, 0},
                                                       {1, 1, 1, 1, 0, 0},
                                                       {1, 1, 1, 0, 0, 0},
                                                       {1, 1, 0, 0, 0, 0},
                                                       {1, 0, 0, 0, 0, 0}}};

    std::vector<module::HardwareIO::muscle_t> muscles;

    module::HardwareIO::muscle_t muscle1 = {
        module::HardwareIO::valve1, module::Sensors::pressuresensor1, module::Sensors::linearpot1, pm_280};

    module::HardwareIO::muscle_t muscle2 = {
        module::HardwareIO::valve2, module::Sensors::pressuresensor2, module::Sensors::linearpot2, pm_280};

    muscles.push_back(muscle1);
    muscles.push_back(muscle2);

    // Make our joints with the previously declared muscles
    module::HardwareIO::joint::OneAxis one_axis_joint(muscles, 1, 0.47, module::MPC::AdaptiveMPC::mpc);

    // Start our ADC DMA
    utility::io::adc_io.Start();
    utility::io::debug.out("Initialisation Finished\n");

    /*******************************************************************************************************************
    **************************************************** Controller ****************************************************
    *******************************************************************************************************************/
    // This is where our 'controller' starts
    // Declare how often we run our loop (Sampling Time)
    float Sampling_time = 50;  // 0.01 T_s
    auto prev_now       = NUClear::clock::now();

    while (1) {

        auto now = NUClear::clock::now();
        // HAL_Delay(1000);
        float time = std::chrono::duration_cast<std::chrono::milliseconds>(NUClear::clock::now() - prev_now).count();

        // utility::io::debug.out("PNEUBot is running %lf\n", time / 1000);

        one_axis_joint.UpdateVelocity();
        // Sampling time controller to handle the periodicity of the code
        if (time >= Sampling_time) {
            utility::io::gpio::led2 = !utility::io::gpio::led2;
            prev_now                = NUClear::clock::now();
            // Time to run our controller again

            utility::io::debug.out("SP %.2f\t", module::Input::controller.GetPosition());

            one_axis_joint.Compute(module::Sensors::linearpot2.GetPosition());

            utility::io::adc_io.Start();

            // utility::io::debug.out("%lf ->\t", time);

            // utility::io::adc_io.PrintSensors();
        }
    }
}

// TODO I think this is how the system needs to work
// Joints know about the muscles, a joint contains all of the muscles it needs to perform. Each muscle has knowledge of
// its respective sensors, pressure, position. A muscle is responsible for reading it's sensors. The joint compute
// function then calls the appropriate MPC function and arranges the inputs how the MPC expects. Something like this