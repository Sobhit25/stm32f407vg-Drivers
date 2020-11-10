################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/i2c_recieve.c \
../Src/sysmem.c 

OBJS += \
./Src/i2c_recieve.o \
./Src/sysmem.o 

C_DEPS += \
./Src/i2c_recieve.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/i2c_recieve.o: ../Src/i2c_recieve.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Inc" -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/i2c_recieve.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/sysmem.o: ../Src/sysmem.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Inc" -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/sysmem.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

