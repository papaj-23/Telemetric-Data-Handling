set(CMAKE_SYSTEM_NAME Generic-ELF)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")

set(TOOLCHAIN_PREFIX arm-none-eabi-)

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)

set(CMAKE_C_STANDARD 11)

set(CPU_OPTS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(COMMON_FLAGS "${CPU_OPTS} -ffunction-sections -fdata-sections -g3 -Wall -Wextra")
set(CMAKE_C_FLAGS   "${COMMON_FLAGS}")
set(CMAKE_ASM_FLAGS "${CPU_OPTS} -x assembler-with-cpp -g3")