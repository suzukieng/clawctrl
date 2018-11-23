cmake_minimum_required(VERSION 2.8.4)
project(clawctrl)

set(CMAKE_CXX_STANDARD 11)

include_directories(Inc)
include_directories(Drivers/STM32L4xx_HAL_Driver/Inc)
include_directories(Drivers/STM32L4xx_HAL_Driver/Inc/Legacy)
include_directories(Drivers/CMSIS/Device/ST/STM32L4xx/Include)
include_directories(Drivers/CMSIS/Include)

add_compile_definitions(USE_HAL_DRIVER)
add_compile_definitions(STM32L412xx)

add_custom_target(clawctrl COMMAND make -C ${clawctrl_SOURCE_DIR}
        CLION_EXE_DIR=${PROJECT_BINARY_DIR})

add_executable (clawctrl_dummy Src/main.c)

