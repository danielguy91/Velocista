################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../source/Velocista.c \
../source/semihost_hardfault.c 

OBJS += \
./source/Velocista.o \
./source/semihost_hardfault.o 

C_DEPS += \
./source/Velocista.d \
./source/semihost_hardfault.d 


# Each subdirectory must supply rules for building sources it contributes
source/%.o: ../source/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DCPU_MK64FN1M0VLL12 -DCPU_MK64FN1M0VLL12_cm4 -DSDK_OS_BAREMETAL -DSERIAL_PORT_TYPE_UART=1 -DSDK_DEBUGCONSOLE=0 -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=0 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\board" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\source" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\drivers" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\utilities" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\component\serial_manager" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\component\uart" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\component\gpio" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\component\timer" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\component\lists" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\component\button" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\component\timer_manager" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\component\led" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\CMSIS" -I"D:\Users\centro electrico\Desktop\UNER\Microcontroladores\K64F\Velocista\device" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="../$(@D)/"=. -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -D__REDLIB__ -fstack-usage -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


