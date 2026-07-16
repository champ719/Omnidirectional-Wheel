set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(ARM_GNU_ROOT "C:/clion-toolchain/arm-gnu-toolchain-13.3.rel1-mingw-w64-i686-arm-none-eabi")
set(ARM_GNU_BIN "${ARM_GNU_ROOT}/bin")

set(CMAKE_C_COMPILER "${ARM_GNU_BIN}/arm-none-eabi-gcc.exe" CACHE FILEPATH "ARM C compiler")
set(CMAKE_ASM_COMPILER "${ARM_GNU_BIN}/arm-none-eabi-gcc.exe" CACHE FILEPATH "ARM assembler")
set(CMAKE_AR "${ARM_GNU_BIN}/arm-none-eabi-ar.exe" CACHE FILEPATH "ARM archiver")
set(CMAKE_RANLIB "${ARM_GNU_BIN}/arm-none-eabi-ranlib.exe" CACHE FILEPATH "ARM ranlib")
set(CMAKE_OBJCOPY "${ARM_GNU_BIN}/arm-none-eabi-objcopy.exe" CACHE FILEPATH "ARM objcopy")
set(CMAKE_SIZE "${ARM_GNU_BIN}/arm-none-eabi-size.exe" CACHE FILEPATH "ARM size")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
