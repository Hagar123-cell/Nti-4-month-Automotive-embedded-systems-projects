################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../EXTI.c \
../GIE_R.c \
../GPIO.c \
../LED.c \
../PUSH_BUTTON.c \
../Timer0.c \
../main.c \
../smallOS.c 

OBJS += \
./EXTI.o \
./GIE_R.o \
./GPIO.o \
./LED.o \
./PUSH_BUTTON.o \
./Timer0.o \
./main.o \
./smallOS.o 

C_DEPS += \
./EXTI.d \
./GIE_R.d \
./GPIO.d \
./LED.d \
./PUSH_BUTTON.d \
./Timer0.d \
./main.d \
./smallOS.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega16 -DF_CPU=1000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


