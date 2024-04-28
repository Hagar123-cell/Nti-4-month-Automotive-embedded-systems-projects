################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../EXTI.c \
../GIE_R.c \
../GPIO.c \
../KEYBAD.c \
../LCD.c \
../LED.c \
../RELAY.c \
../TIMER1_prog.c \
../Timer0.c \
../ULTRASONIC.c \
../main.c 

OBJS += \
./EXTI.o \
./GIE_R.o \
./GPIO.o \
./KEYBAD.o \
./LCD.o \
./LED.o \
./RELAY.o \
./TIMER1_prog.o \
./Timer0.o \
./ULTRASONIC.o \
./main.o 

C_DEPS += \
./EXTI.d \
./GIE_R.d \
./GPIO.d \
./KEYBAD.d \
./LCD.d \
./LED.d \
./RELAY.d \
./TIMER1_prog.d \
./Timer0.d \
./ULTRASONIC.d \
./main.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: AVR Compiler'
	avr-gcc -Wall -g2 -gstabs -O0 -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -std=gnu99 -funsigned-char -funsigned-bitfields -mmcu=atmega16 -DF_CPU=1000000UL -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


