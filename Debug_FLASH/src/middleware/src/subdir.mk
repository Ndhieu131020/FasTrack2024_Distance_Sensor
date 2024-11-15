################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/middleware/src/MID_CAN_Interface.c \
../src/middleware/src/MID_Clock_Interface.c \
../src/middleware/src/MID_GPIO_Interface.c \
../src/middleware/src/MID_Notification_Manager.c \
../src/middleware/src/MID_Sensor_Interface.c \
../src/middleware/src/MID_Timer_Interface.c 

OBJS += \
./src/middleware/src/MID_CAN_Interface.o \
./src/middleware/src/MID_Clock_Interface.o \
./src/middleware/src/MID_GPIO_Interface.o \
./src/middleware/src/MID_Notification_Manager.o \
./src/middleware/src/MID_Sensor_Interface.o \
./src/middleware/src/MID_Timer_Interface.o 

C_DEPS += \
./src/middleware/src/MID_CAN_Interface.d \
./src/middleware/src/MID_Clock_Interface.d \
./src/middleware/src/MID_GPIO_Interface.d \
./src/middleware/src/MID_Notification_Manager.d \
./src/middleware/src/MID_Sensor_Interface.d \
./src/middleware/src/MID_Timer_Interface.d 


# Each subdirectory must supply rules for building sources it contributes
src/middleware/src/%.o: ../src/middleware/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Standard S32DS C Compiler'
	arm-none-eabi-gcc "@src/middleware/src/MID_CAN_Interface.args" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


