################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f407_gpio_drivers.c \
../drivers/Src/stm32f407_i2c_drivers.c \
../drivers/Src/stm32f407_rcc_drivers.c \
../drivers/Src/stm32f407_spi_drivers.c \
../drivers/Src/stm32f407_usart_drivers.c 

OBJS += \
./drivers/Src/stm32f407_gpio_drivers.o \
./drivers/Src/stm32f407_i2c_drivers.o \
./drivers/Src/stm32f407_rcc_drivers.o \
./drivers/Src/stm32f407_spi_drivers.o \
./drivers/Src/stm32f407_usart_drivers.o 

C_DEPS += \
./drivers/Src/stm32f407_gpio_drivers.d \
./drivers/Src/stm32f407_i2c_drivers.d \
./drivers/Src/stm32f407_rcc_drivers.d \
./drivers/Src/stm32f407_spi_drivers.d \
./drivers/Src/stm32f407_usart_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32f407_gpio_drivers.o: ../drivers/Src/stm32f407_gpio_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Inc" -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_gpio_drivers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_i2c_drivers.o: ../drivers/Src/stm32f407_i2c_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Inc" -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_i2c_drivers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_rcc_drivers.o: ../drivers/Src/stm32f407_rcc_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Inc" -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_rcc_drivers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_spi_drivers.o: ../drivers/Src/stm32f407_spi_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Inc" -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_spi_drivers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
drivers/Src/stm32f407_usart_drivers.o: ../drivers/Src/stm32f407_usart_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Inc" -I"D:/Embedded C/My_Workspace/target/stm32f407_drivers/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407_usart_drivers.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

