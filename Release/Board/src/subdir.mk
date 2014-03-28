################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_UPPER_SRCS += \
../Board/src/FIRE_RTC_count.C 

C_SRCS += \
../Board/src/FIRE_OV7725_Eagle.c \
../Board/src/FIRE_camera.c \
../Board/src/FIRE_key.c 

OBJS += \
./Board/src/FIRE_OV7725_Eagle.o \
./Board/src/FIRE_RTC_count.o \
./Board/src/FIRE_camera.o \
./Board/src/FIRE_key.o 

C_DEPS += \
./Board/src/FIRE_OV7725_Eagle.d \
./Board/src/FIRE_camera.d \
./Board/src/FIRE_key.d 

C_UPPER_DEPS += \
./Board/src/FIRE_RTC_count.d 


# Each subdirectory must supply rules for building sources it contributes
Board/src/%.o: ../Board/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	arm-none-eabi-gcc -I"F:\Armworkspace\SmartcarProject\inc" -I"F:\Armworkspace\libsmartcarcpp\inc" -I"F:\Armworkspace\libsmartcarcpp\src" -I"F:\Armworkspace\SmartcarProject\src" -I"F:\Armworkspace\libk60base\inc" -I"F:\Armworkspace\libk60base\src" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Board/src/%.o: ../Board/src/%.C
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	arm-none-eabi-g++ -I"F:\Armworkspace\SmartcarProject\inc" -I"F:\Armworkspace\libsmartcarcpp\inc" -I"F:\Armworkspace\libsmartcarcpp\src" -I"F:\Armworkspace\SmartcarProject\src" -I"F:\Armworkspace\libk60base\inc" -I"F:\Armworkspace\libk60base\src" -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


