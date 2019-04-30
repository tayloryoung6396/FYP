# Make sure the toolchain file has been supplied.
if(NOT CMAKE_CROSSCOMPILING)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/Toolchain.cmake")
endif(NOT CMAKE_CROSSCOMPILING)

# Default to building using release
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# This must be after the crosscompile so it correctly uses the crosscompiler code
cmake_minimum_required(VERSION 3.11)
project(PNEUbots C CXX ASM)

# Hide some useless cmake variables
mark_as_advanced(CMAKE_INSTALL_PREFIX)

if(CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  add_compile_options(-gdwarf-2)
endif()

# Print the debugging in colour
add_compile_options(-fdiagnostics-color=always)

# Suppress warnings about "XXX changed in GCC 7.1"
add_compile_options(-Wno-psabi)

# Build the STM library
add_subdirectory(src/stm32)

# Build NUClear library
add_subdirectory(src/nuclear)

# Build the shared code library
add_subdirectory(src/shared)

# Build the shared code library
add_subdirectory(src/module)

# Add the main executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME}.elf)
target_link_libraries(${PROJECT_NAME} stm32 nuclear)
target_link_libraries(${PROJECT_NAME} stm32 utility module)

# Create a map and cross reference file for locating where things live on the device
target_link_options(${PROJECT_NAME} PRIVATE -Wl,-Map=${PROJECT_NAME}.map,--cref)

# Create the .bin and .hex file.
add_custom_command(
  OUTPUT ${PROJECT_NAME}.bin
  COMMAND
    ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
  DEPENDS ${PROJECT_NAME}
)
add_custom_command(
  OUTPUT ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
  DEPENDS ${PROJECT_NAME}
)
add_custom_target(bin ALL DEPENDS ${PROJECT_NAME}.bin)
add_custom_target(hex ALL DEPENDS ${PROJECT_NAME}.hex)