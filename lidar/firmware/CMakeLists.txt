cmake_minimum_required(VERSION 3.20)

project(LD19Bench C ASM)

if(NOT CMAKE_TOOLCHAIN_FILE)
  message(FATAL_ERROR "Pass -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(APP_VERSION "LD19_BENCH_F407VE_0.3.3" CACHE STRING "firmware identity string")
set(PROTOCOL_VERSION 2 CACHE STRING "host protocol version")

set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/linker/STM32F407VE_bench.ld")
set(VENDOR "${CMAKE_CURRENT_SOURCE_DIR}/vendor")

add_compile_definitions(
  STM32F407xx
  USE_HAL_DRIVER
  LD19_BENCH_F407VE=1
  APP_VERSION="${APP_VERSION}"
  PROTOCOL_VERSION=${PROTOCOL_VERSION}
  HSE_VALUE=25000000U
)

include_directories(
  Core/Inc
  BSP/Inc
  APP/Inc
  ${VENDOR}/CMSIS/Include
  ${VENDOR}/CMSIS/Device/ST/STM32F4xx/Include
  ${VENDOR}/STM32F4xx_HAL_Driver/Inc
)

set(APP_SOURCES
  APP/Src/ld19_parser.c
  APP/Src/scan_builder.c
  APP/Src/bridge.c
  APP/Src/protocol.c
  APP/Src/status.c
  APP/Src/features.c
  APP/Src/motion_history.c
  APP/Src/pose_match.c
  APP/Src/shadow_localizer.c
  APP/Src/app.c
)
set(BSP_SOURCES
  BSP/Src/board_clock.c
  BSP/Src/timebase.c
  BSP/Src/usart1.c
  BSP/Src/usart2.c
  BSP/Src/vehicle_uart.c
  BSP/Src/spi1.c
  BSP/Src/i2c1.c
  BSP/Src/storage_diag.c
)
set(CORE_SOURCES
  Core/Src/main.c
  Core/Src/system_stm32f4xx.c
  Core/Src/stm32f4xx_it.c
  Core/Src/syscalls.c
)
set(VENDOR_SOURCES
  ${VENDOR}/CMSIS/Device/ST/STM32F4xx/Source/Templates/startup_stm32f407xx.s
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
  ${VENDOR}/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
)

add_executable(LD19Bench ${APP_SOURCES} ${BSP_SOURCES} ${CORE_SOURCES} ${VENDOR_SOURCES})

# ST HAL/CMSIS sources are vendor code: keep warnings on but tolerate the
# unused-parameter noise that is normal for a cross-device HAL driver.
set_source_files_properties(${VENDOR_SOURCES} PROPERTIES COMPILE_OPTIONS
  "-Wno-unused-parameter;-Wno-sign-compare;-Wno-missing-field-initializers")

target_link_options(LD19Bench PRIVATE
  -T${LINKER_SCRIPT}
  -Wl,-Map=${CMAKE_BINARY_DIR}/LD19Bench.map
  -Wl,--print-memory-usage
  -Wl,--no-warn-rwx-segments
)
target_link_libraries(LD19Bench PRIVATE m)

set_target_properties(LD19Bench PROPERTIES LINK_DEPENDS "${LINKER_SCRIPT}")
# Bare-metal CMake defaults to no executable suffix; force the conventional .elf
# so build.json/objcopy/flashing agree on one filename.
set_target_properties(LD19Bench PROPERTIES SUFFIX ".elf")

add_custom_command(TARGET LD19Bench POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:LD19Bench> ${CMAKE_BINARY_DIR}/LD19Bench.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:LD19Bench> ${CMAKE_BINARY_DIR}/LD19Bench.hex
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:LD19Bench>
  COMMENT "Generating BIN/HEX and size report"
)
