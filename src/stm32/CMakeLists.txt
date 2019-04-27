# Set this to the files used in the generated makefile
set(
  src
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pcd_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_ll_usb.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_i2c_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
  Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
  Src/main.c
  Src/usb_device.c
  Src/usbd_conf.c
  Src/usbd_desc.c
  Src/usbd_cdc_if.c
  Src/stm32h7xx_it.c
  Src/stm32h7xx_hal_msp.c
  Src/system_stm32h7xx.c
  Src/gpio.c
  Src/i2c.c
  Src/spi.c
  Src/usart.c
  Src/tim.c startup_stm32h753xx.s
)

# Make the library as a static library
add_library(stm32 OBJECT ${src})

# Make the compile definitions
target_compile_definitions(stm32 PUBLIC USE_HAL_DRIVER)
target_compile_definitions(stm32 PUBLIC STM32H753xx)

# Include the directories as private unless they are needed elsewhere
target_include_directories(
  stm32
  PUBLIC Drivers/STM32H7xx_HAL_Driver/Inc
  PUBLIC Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
  PUBLIC Middlewares/ST/STM32_USB_Device_Library/Core/Inc
  PUBLIC Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
  PUBLIC Drivers/CMSIS/Device/ST/STM32H7xx/Include
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
  -T${CMAKE_CURRENT_SOURCE_DIR}/STM32H753VITx_FLASH.ld
)
