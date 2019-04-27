# This must be after the crosscompile so it correctly uses the crosscompiler code
cmake_minimum_required(VERSION 3.10)
project(PNEUbots)

# Hide some useless cmake variables
mark_as_advanced(CMAKE_INSTALL_PREFIX)

set(
	Src
	src/main.cpp
	src/module/HardwareIO/Joint/OneAxis/OneAxis.cpp
	src/module/HardwareIO/Joint/TwoAxis/TwoAxis.cpp
	src/module/HardwareIO/Muscle/Muscle.cpp
	src/module/HardwareIO/Valve/Valve.cpp
	src/module/Sensors/LinearPotentiometer/LinearPotentiometer.cpp
	src/module/Sensors/PressureSensor/PressureSensor.cpp
	src/shared/utility/PID/PID.cpp
	src/shared/utility/ADC/ADC.cpp
	src/shared/utility/motion/InverseKinematics.cpp
	src/shared/utility/clock/clock.cpp
	
)

# Add the main executable
add_executable(${PROJECT_NAME} ${Src})
