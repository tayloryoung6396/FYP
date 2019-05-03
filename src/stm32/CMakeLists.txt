# Set this to the files used in the generated makefile
set(
  src
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_adc_ex.c
  # Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c
  # Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c
  # Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
  # Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c
  # Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c 

  Src/main.c
  Src/stm32f7xx_it.c
  Src/stm32f7xx_hal_msp.c
  Src/system_stm32f7xx.c
  Src/gpio.c
  # Src/i2c.c
  Src/usart.c
  # Src/usb_otg.c
  Src/adc.c
  Src/tim.c startup_stm32f722xx.s
)

# Make the library as a static library
add_library(stm32 OBJECT ${src})

# Make the compile definitions
target_compile_definitions(stm32 PUBLIC USE_HAL_DRIVER)
target_compile_definitions(stm32 PUBLIC STM32F722xx)

# Include the directories as private unless they are needed elsewhere
target_include_directories(
  stm32
  PUBLIC Drivers/STM32F7xx_HAL_Driver/Inc
  PUBLIC Drivers/STM32F7xx_HAL_Driver/Inc/Legacy
  PUBLIC Drivers/CMSIS/Device/ST/STM32F7xx/Include
  PUBLIC Drivers/CMSIS/Include
  PUBLIC Drivers/CMSIS/Include
  PUBLIC Inc
)

# When linking to this library, ensure that it uses the linker script
target_link_options(
  stm32
  PUBLIC
  -specs=nosys.specs
  -specs=nano.specs
  -u
  _printf_float
  -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F722ZETx_FLASH.ld
)
