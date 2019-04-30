#include "main.hpp"
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "module/HardwareIO/Joint/LinearAxis/LinearAxis.hpp"
#include "module/HardwareIO/Joint/OneAxis/OneAxis.hpp"
#include "module/HardwareIO/Joint/TwoAxis/TwoAxis.hpp"
#include "module/HardwareIO/Muscle/Muscle.hpp"
#include "module/HardwareIO/Valve/Valve.hpp"
#include "module/Sensors/LinearPotentiometer/LinearPotentiometer.hpp"
#include "module/Sensors/PressureSensor/PressureSensor.hpp"
#include "nuclear/src/clock.hpp"
#include "shared/utility/PID/PID.hpp"
#include "shared/utility/io/adc.hpp"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_it.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"


extern "C" {
void SystemClock_Config(void);
}

int main() {
    std::cout << "Welcome to PNEUbot" << std::endl;

    // Initialise the HAL driver
    HAL_Init();

    // Configure the system clock
    SystemClock_Config();
    utility::clock::initialise();

    // Initialize all configured peripherals
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC3_Init();
    MX_I2C1_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();

    std::vector<module::HardwareIO::muscle_t> muscles;

    module::HardwareIO::muscle_t muscle1 = {module::HardwareIO::valve1,
                                            module::Sensors::pressuresensor1,
                                            module::Sensors::linearpot1,
                                            shared::utility::pid1,
                                            0.1};

    muscles.push_back(muscle1);

    module::HardwareIO::joint::LinearAxis linear_muscle(muscles);

    while (1) {
        // Decide where i want the joint positions
        // Read ADC
        // Call all joint funtions to act upon the request
        // linear_muscle.Compute(100);


        NUClear::clock::now();
    }
}