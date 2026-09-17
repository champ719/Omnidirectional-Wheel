#!/bin/sh
# 用 CMakeLists.txt 里的真实 flag 做单文件语法检查
INC="-ICore/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include \
-IDrivers/CMSIS/DSP/Include \
-IDrivers/CMSIS/DSP/PrivateInclude \
-IMiddlewares/Third_Party/FreeRTOS/Source/include \
-IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
-IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F \
-IUser/Application \
-IUser/Control \
-IUser/Communication \
-IUser/Device \
-IUser/math \
-IUser/BSP/inc \
-IUser/IMU/inc \
-IUser/IMU/BMI088/inc \
-IUser/IMU/Algorithm/inc"

DEF="-DUSE_HAL_DRIVER -DSTM32F407xx -DARM_MATH_CM4 -DHSE_VALUE=12000000U"
MCU="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"

for f in "$@"; do
    echo "########## $f ##########"
    arm-none-eabi-gcc -fsyntax-only -std=gnu11 -Wall -Wextra $MCU $DEF $INC "$f" 2>&1
    echo "---- exit=$? ----"
done
