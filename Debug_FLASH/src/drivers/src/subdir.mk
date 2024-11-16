################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/drivers/src/DRV_S32K144_ADC.c \
../src/drivers/src/DRV_S32K144_FLEXCAN.c \
../src/drivers/src/DRV_S32K144_GPIO.c \
../src/drivers/src/DRV_S32K144_LPIT.c \
../src/drivers/src/DRV_S32K144_MCU.c \
../src/drivers/src/DRV_S32K144_NVIC.c \
../src/drivers/src/DRV_S32K144_PORT.c 

OBJS += \
./src/drivers/src/DRV_S32K144_ADC.o \
./src/drivers/src/DRV_S32K144_FLEXCAN.o \
./src/drivers/src/DRV_S32K144_GPIO.o \
./src/drivers/src/DRV_S32K144_LPIT.o \
./src/drivers/src/DRV_S32K144_MCU.o \
./src/drivers/src/DRV_S32K144_NVIC.o \
./src/drivers/src/DRV_S32K144_PORT.o 

C_DEPS += \
./src/drivers/src/DRV_S32K144_ADC.d \
./src/drivers/src/DRV_S32K144_FLEXCAN.d \
./src/drivers/src/DRV_S32K144_GPIO.d \
./src/drivers/src/DRV_S32K144_LPIT.d \
./src/drivers/src/DRV_S32K144_MCU.d \
./src/drivers/src/DRV_S32K144_NVIC.d \
./src/drivers/src/DRV_S32K144_PORT.d 


# Each subdirectory must supply rules for building sources it contributes
src/drivers/src/%.o: ../src/drivers/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C Compiler'
	arm-none-eabi-gcc "@src/drivers/src/DRV_S32K144_ADC.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


