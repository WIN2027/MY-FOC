cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME FOC)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# 强制使用 -O2 优化（覆盖 Debug 的 -O0）
set(CMAKE_C_FLAGS_DEBUG "-O3 -g3")
set(CMAKE_C_FLAGS_RELEASE "-O3 -g0")
# 添加宏，启用 CMSIS-DSP 硬件 FPU 加速
add_compile_definitions(ARM_MATH_CM4)


# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/main.c
        DOC/Src/Init.c
        DOC/Src/FreeRTOS_Task.c
        DOC/Src/Back.c
        DOC/Src/AppFOC.c
        DOC/Src/UART.c
        DOC/Src/ADC.c
        DOC/Src/DRV8320S.c
        Modules/FOC_ADC/FOC_ADC.c
        Modules/FOC_CORE/FOC_CORE.c
        Modules/FOC_CORE/FOC_PORT_STM32G4.c
        Modules/UART_Protocol/Protocol.c
        Modules/UART_Protocol/Protocol_HAL.c
        Middlewares/ST/ARM/DSP/Inc/arm_math.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        DOC/Inc
        Modules/FOC_ADC
        Modules/FOC_CORE
        Modules/UART_Protocol
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    PROTOCOL_DEFER_HANDLER=1
    FOC_USE_CCM_RAM
    ADC_FILTER_TEXT_CCM
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
target_link_options(FOC PRIVATE -u _printf_float)

target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Winline)