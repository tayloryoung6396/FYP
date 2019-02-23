# This must be after the crosscompile so it correctly uses the crosscompiler code
cmake_minimum_required(VERSION 3.10)
project(PNEUbot)

set(
  Src
  src/main.cpp
src/module/LinearPotentiometer/LinearPotentiometer.cpp
src/module/PID/PID.cpp
src/module/PressureSensor/PressureSensor.cpp
src/module/Valve/Valve.cpp
)

# Add the main executable
add_executable(${PROJECT_NAME} ${Src})
