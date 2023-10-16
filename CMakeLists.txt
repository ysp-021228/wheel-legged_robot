#THIS FILE IS AUTO GENERATED FROM THE TEMPLATE! DO NOT CHANGE!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.19)

# specify cross compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(demo1 C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

include_directories(Inc Drivers/STM32F4xx_HAL_Driver/Inc
        segger/SEGGER
        segger/FreeRTOSV10
        segger/Config
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
        Middlewares/Third_Party/FreeRTOS/Source/include
        Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
        Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
        Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/CMSIS/Include
        bsp
        component/algorithm
        component/controller
        component/devices
        component/support
        application/
        )

add_definitions(-DUSE_HAL_DRIVER -DSTM32F407xx)

file(GLOB_RECURSE SOURCES "segger/*.*" "startup/*.*" "Middlewares/*.*" "Drivers/*.*" "Src/*.*" "bsp/*.*" "component/*.*" "application/*.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407IGHx_FLASH.ld)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})
LINK_LIBRARIES(m)
add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT}
        component/controller/PID.h
        application/Referee.c
        application/Referee.h
        application/Calibrate.c
        application/Calibrate.h
        component/algorithm/AHRS_MiddleWare.c
        component/algorithm/AHRS_MiddleWare.h
        component/algorithm/AHRS.c
        component/algorithm/AHRS.h
        component/algorithm/ramp.c
        component/algorithm/ramp.h
        application/Can_send.c
        application/Can_send.h
        application/Detection.c
        application/Detection.h)

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
