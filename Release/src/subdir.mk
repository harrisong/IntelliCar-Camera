################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/OV7725.c \
../src/SCCB.c \
../src/isr.c \
../src/main.c 

OBJS += \
./src/OV7725.o \
./src/SCCB.o \
./src/isr.o \
./src/main.o 

C_DEPS += \
./src/OV7725.d \
./src/SCCB.d \
./src/isr.d \
./src/main.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -I"F:\Armworkspace\SmartcarProject\inc" -I"F:\Armworkspace\libsmartcarcpp\inc" -I"F:\Armworkspace\libsmartcarcpp\src" -I"F:\Armworkspace\SmartcarProject\src" -I"F:\Armworkspace\libk60base\inc" -I"F:\Armworkspace\libk60base\src" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


