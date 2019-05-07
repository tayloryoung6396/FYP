set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CROSS_COMPILE "arm-none-eabi-")

if(CMAKE_VERSION VERSION_LESS 3.6)
  include(CMakeForceCompiler)
  cmake_force_c_compiler("${CROSS_COMPILE}gcc" GNU)
  cmake_force_cxx_compiler("${CROSS_COMPILE}g++" GNU)
else()
  # Use add_library() with the STATIC option to name the source file in the generated project. This avoids running the
  # linker and is intended for use with cross-compiling toolchains that cannot link without custom flags or linker
  # scripts.
  set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
  set(CMAKE_C_COMPILER "${CROSS_COMPILE}gcc")
  set(CMAKE_CXX_COMPILER "${CROSS_COMPILE}g++")
endif()

set(CMAKE_AR "${CROSS_COMPILE}ar" CACHE STRING "" FORCE)
set(CMAKE_ASM_COMPILER "${CROSS_COMPILE}gcc" CACHE STRING "" FORCE)
set(CMAKE_OBJCOPY "${CROSS_COMPILE}objcopy" CACHE STRING "" FORCE)
set(CMAKE_OBJDUMP "${CROSS_COMPILE}objdump" CACHE STRING "" FORCE)
set(CMAKE_RANLIB "${CROSS_COMPILE}ranlib" CACHE STRING "" FORCE)
set(CMAKE_NM "${CROSS_COMPILE}nm" CACHE STRING "" FORCE)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_ASM_FLAGS "-x assembler-with-cpp")

# Set the CPU flags
set(CPU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard)
add_compile_options(${CPU_FLAGS})
add_link_options(${CPU_FLAGS})

# Flags to reduce the size of the final object file
add_compile_options(-fdata-sections -ffunction-sections)
add_link_options(-Wl,--gc-sections)