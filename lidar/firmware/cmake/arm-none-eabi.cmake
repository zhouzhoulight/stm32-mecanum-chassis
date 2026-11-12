# Cross toolchain file for STM32F407VET6 (arm-none-eabi-gcc).
# Pass -DTOOLCHAIN_ROOT=... or -DARM_GCC=... to override; no hard-coded user paths
# are required (build.ps1 can also set these from its parameters).
cmake_minimum_required(VERSION 3.20)

if(NOT DEFINED ARM_GCC)
  if(DEFINED ENV{LOCALAPPDATA})
    # common STM32CubeCLT default (not required)
  endif()
endif()

if(DEFINED ARM_GCC)
  file(TO_CMAKE_PATH "${ARM_GCC}" _ARM_GCC)
  set(CMAKE_C_COMPILER "${_ARM_GCC}")
  set(CMAKE_ASM_COMPILER "${_ARM_GCC}")
  get_filename_component(_arm_bin "${_ARM_GCC}" DIRECTORY)
  if(WIN32 OR CMAKE_HOST_WIN32)
    set(_exe ".exe")
  else()
    set(_exe "")
  endif()
  set(CMAKE_OBJCOPY "${_arm_bin}/arm-none-eabi-objcopy${_exe}" CACHE FILEPATH "")
  set(CMAKE_SIZE "${_arm_bin}/arm-none-eabi-size${_exe}" CACHE FILEPATH "")
else()
  set(CMAKE_C_COMPILER arm-none-eabi-gcc)
  set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
  set(CMAKE_OBJCOPY arm-none-eabi-objcopy CACHE FILEPATH "")
  set(CMAKE_SIZE arm-none-eabi-size CACHE FILEPATH "")
endif()

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CPU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS_INIT "${CPU_FLAGS} -std=c11 -Wall -Wextra -Werror -ffunction-sections -fdata-sections -fno-common -fstack-usage")
set(CMAKE_ASM_FLAGS_INIT "${CPU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${CPU_FLAGS} -Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/LD19Bench.map -specs=nano.specs -specs=nosys.specs")
