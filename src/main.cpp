#include "main.hpp"
#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include "HardwareIO/Joint/LinearAxis/LinearAxis.hpp"
#include "HardwareIO/Joint/OneAxis/OneAxis.hpp"
#include "HardwareIO/Joint/TwoAxis/TwoAxis.hpp"
#include "HardwareIO/Muscle/Muscle.hpp"
#include "HardwareIO/Valve/Valve.hpp"
#include "Sensors/LinearPotentiometer/LinearPotentiometer.hpp"
#include "Sensors/PressureSensor/PressureSensor.hpp"
#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "nuclear/src/clock.hpp"
#include "stm32f7xx_hal_conf.h"
#include "stm32f7xx_it.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
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
    MX_I2C1_Init();
    MX_USART3_UART_Init();
    MX_USB_OTG_FS_PCD_Init();

    utility::io::debug.out("Welcome to PNEUbot\n");
    utility::io::adc_io.GetSensors(1);

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
        linear_muscle.Compute(100);


        NUClear::clock::now();
    }
}