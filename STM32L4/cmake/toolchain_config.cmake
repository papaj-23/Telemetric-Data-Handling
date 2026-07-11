set(CMAKE_SYSTEM_NAME Generic-ELF)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(TOOLCHAIN_PREFIX arm-none-eabi-)

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)

set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE    ${TOOLCHAIN_PREFIX}size)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)

set(CMAKE_C_STANDARD 11)

set(CPU_FLAGS " -mcpu=cortex-m4 -mthumb")
set(FPU_FLAGS " -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(COMMON_WARN_FLAGS " -ffunction-sections -fdata-sections -g3 -Wall -Wextra")

set(CMAKE_C_FLAGS   "${CPU_FLAGS}${FPU_FLAGS}${COMMON_WARN_FLAGS}${OPT_FLAGS}")

set(CMAKE_ASM_FLAGS "${CPU_FLAGS}${FPU_FLAGS} -x assembler-with-cpp -g3")

set(CMAKE_EXE_LINKER_FLAGS "${CPU_FLAGS}${FPU_FLAGS} -Wl,--gc-sections")