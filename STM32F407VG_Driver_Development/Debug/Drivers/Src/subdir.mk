################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f407vg_gpio_driver.c \
../Drivers/Src/stm32f407vg_spi_driver.c 

OBJS += \
./Drivers/Src/stm32f407vg_gpio_driver.o \
./Drivers/Src/stm32f407vg_spi_driver.o 

C_DEPS += \
./Drivers/Src/stm32f407vg_gpio_driver.d \
./Drivers/Src/stm32f407vg_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/stm32f407vg_gpio_driver.o: ../Drivers/Src/stm32f407vg_gpio_driver.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"/home/sobhit25/STM32CubeIDE/MyWorkSpace/target/STM32F407VG_Driver_Development/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f407vg_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/Src/stm32f407vg_spi_driver.o: ../Drivers/Src/stm32f407vg_spi_driver.c Drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -I"/home/sobhit25/STM32CubeIDE/MyWorkSpace/target/STM32F407VG_Driver_Development/Drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f407vg_spi_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

