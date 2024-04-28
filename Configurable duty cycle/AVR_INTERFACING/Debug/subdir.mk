################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DC_MOTOR.c \
../EXTI.c \
../GIE_R.c \
../GPIO.c \
../KEYBAD.c \
../LCD.c \
../Timer0.c \
../main.c 

OBJS += \
./DC_MOTOR.o \
./EXTI.o \
./GIE_R.o \
./GPIO.o \
./KEYBAD.o \
./LCD.o \
./Timer0.o \
./main.o 

C_DEPS += \
./DC_MOTOR.d \
./EXTI.d \
./GIE_R.d \
./GPIO.d \
./KEYBAD.d \
./LCD.d \
./Timer0.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega16 -DF_CPU=1000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


