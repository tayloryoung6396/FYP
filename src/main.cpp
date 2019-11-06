#include "main.hpp"
#include <Eigen/Core>
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
#include "PID/PID.hpp"
#include "SMC/SMC.hpp"
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
    utility::io::debug.error("\n");
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

    utility::io::debug.out("\033[3J");
    utility::io::debug.out("\014");
    utility::io::debug.out("\033[3J");
    utility::io::debug.out("\014");
    utility::io::debug.out("\e[0m");

    utility::io::debug.out("Welcome to PNEUbot\n");

    // We need our setpoint potentiometer
    // auto Setpoint = module::Input::controller.GetPosition();


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

    constexpr float p00 = (76.27) * 10e-35;       // (76.25, 76.29)
    constexpr float p10 = (1.515e+12) * 10e-35;   // (-1.512e+13, 1.815e+13)
    constexpr float p01 = (-1.515e+12) * 10e-35;  // (-1.815e+13, 1.512e+13)
    constexpr float p20 = (7.645e+12) * 10e-35;   // (-3.98e+12, 1.927e+13)
    constexpr float p11 = (-1.984e+13) * 10e-35;  // (-3.816e+13, -1.518e+12)
    constexpr float p02 = (1.219e+13) * 10e-35;   // (-2.637e+12, 2.702e+13)
    constexpr float p30 = (-1.535e+13) * 10e-35;  // (-3.118e+13, 4.877e+11)
    constexpr float p21 = (3.063e+13) * 10e-35;   // (9.519e+12, 5.174e+13)
    constexpr float p12 = (-1.087e+13) * 10e-35;  // (-3.451e+13, 1.277e+13)
    constexpr float p03 = (-4.412e+12) * 10e-35;  // (-2.166e+13, 1.284e+13)
    constexpr float p40 = (2.531e+13) * 10e-35;   // (1.529e+13, 3.532e+13)
    constexpr float p31 = (-2.712e+13) * 10e-35;  // (-3.913e+13, -1.511e+13)
    constexpr float p22 = (-1.771e+13) * 10e-35;  // (-2.676e+13, -8.651e+12)
    constexpr float p13 = (1.455e+13) * 10e-35;   // (9.587e+11, 2.814e+13)
    constexpr float p04 = (4.974e+12) * 10e-35;   // (-5.777e+12, 1.572e+13)
    constexpr float p50 = (-1.836e+12) * 10e-35;  // (-4.701e+12, 1.03e+12)
    constexpr float p41 = (3.682e+12) * 10e-35;   // (1.3e+12, 6.063e+12)
    constexpr float p32 = (2.06e+12) * 10e-35;    // (-9.788e+11, 5.098e+12)
    constexpr float p23 = (-6.712e+12) * 10e-35;  // (-9.919e+12, -3.506e+12)
    constexpr float p14 = (1.642e+12) * 10e-35;   // (-9.098e+11, 4.195e+12)
    constexpr float p05 = (1.164e+12) * 10e-35;   // (-1.678e+12, 4.006e+12)

    // clang-format off

    Eigen::Matrix<float, 6, 6> F_ce;
    F_ce <<  p00, p01, p02, p03, p04, p05,
             p10, p11, p12, p13, p14, 0,
             p20, p21, p22, p23, 0,   0,
             p30, p31, p32, 0,   0,   0,
             p40, p41, 0,   0,   0,   0,
             p50, 0,   0,   0,   0,   0;   
    const module::HardwareIO::muscle_properties_t pm_280 = {0.25, 0.108, 0.433, 2.6167e-9, 298.15, 298.15, 0.0007, {1.6e-4, -6.4e-4, 5.5e-4, 0.8e-4}, F_ce};
    // clang-format on

    std::vector<module::HardwareIO::muscle_t> muscles;

    module::HardwareIO::muscle_t muscle1 = {
        module::HardwareIO::valve1, module::Sensors::pressuresensor1, module::Sensors::linearpot1, pm_280};

    module::HardwareIO::muscle_t muscle2 = {
        module::HardwareIO::valve2, module::Sensors::pressuresensor2, module::Sensors::linearpot2, pm_280};

    muscles.push_back(muscle1);
    muscles.push_back(muscle2);

    // Make our joints with the previously declared muscles
    // MPC Controller
    // module::HardwareIO::joint::OneAxis<module::MPC::AdaptiveMPC::AdaptiveMPC> one_axis_joint(
    //     muscles, 1, 0.047, module::MPC::AdaptiveMPC::optimizer1);
    // SMC Controller
    // module::HardwareIO::joint::OneAxis<module::SMC::SMC> one_axis_joint(muscles, 1, 0.47, module::SMC::smc);
    // PID Controller
    module::HardwareIO::joint::OneAxis<module::PID::PID> one_axis_joint(muscles, 1, 0.47, module::PID::pid);

    // Start our ADC DMA
    utility::io::adc_io.Start();
    utility::io::debug.out("Initialisation Finished\n");

    one_axis_joint.Compute(0);

    /*******************************************************************************************************************
    **************************************************** Controller ****************************************************
    *******************************************************************************************************************/
    // This is where our 'controller' starts
    // float Sampling_time = 50;  // 0.01 T_s
    // int i               = 0;
    // time_start          = NUClear::clock::now();  // TODO Remove
    // auto prev_now       = NUClear::clock::now();

    // while (1) {

    //     // auto now   = NUClear::clock::now();
    //     float time = std::chrono::duration_cast<std::chrono::milliseconds>(NUClear::clock::now() - prev_now).count();

    //     // utility::io::debug.out("PNEUBot is running %lf\n", time / 1000);

    //     one_axis_joint.UpdateVelocity();

    //     // Sampling time controller to handle the periodicity of the code
    //     if (time >= Sampling_time) {  // && time < Sampling_time * 2) {
    //         utility::io::gpio::led2 = !utility::io::gpio::led2;
    //         prev_now                = NUClear::clock::now();
    //         // Time to run our controller again
    //         // utility::io::debug.out("SP %.2f | ", module::Input::controller.GetPosition());

    //         one_axis_joint.Compute(module::Input::controller.GetPosition());

    //         utility::io::adc_io.Start();
    //         Error_Handler();

    //         // utility::io::debug.out("%lf ->\t", time);

    //         // utility::io::adc_io.PrintSensors();
    //         i++;
    //     }
    //     // else if (time >= Sampling_time) {
    //     //     utility::io::debug.out("Too slow %lf\n", time);
    //     //     Error_Handler();
    //     // }
    //     if (i >= 500) {
    //         time = std::chrono::duration_cast<std::chrono::milliseconds>(NUClear::clock::now() - time_start).count();
    //         utility::io::debug.out("TIME %lf, Avg %lf\n", time, time / 500.0);
    //         Error_Handler();
    //     }
    // }
}

// TODO I think this is how the system needs to work
// Joints know about the muscles, a joint contains all of the muscles it needs to perform. Each muscle has knowledge of
// its respective sensors, pressure, position. A muscle is responsible for reading it's sensors. The joint compute
// function then calls the appropriate MPC function and arranges the inputs how the MPC expects. Something like this