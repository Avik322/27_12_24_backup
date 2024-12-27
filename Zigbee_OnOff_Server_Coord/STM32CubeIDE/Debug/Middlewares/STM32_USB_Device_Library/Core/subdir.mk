################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/gibutakov/STM32CubeIDE/workspace_1.16.1/Zigbee_OnOff_Server_Coord/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
C:/Users/gibutakov/STM32CubeIDE/workspace_1.16.1/Zigbee_OnOff_Server_Coord/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
C:/Users/gibutakov/STM32CubeIDE/workspace_1.16.1/Zigbee_OnOff_Server_Coord/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c 

OBJS += \
./Middlewares/STM32_USB_Device_Library/Core/usbd_core.o \
./Middlewares/STM32_USB_Device_Library/Core/usbd_ctlreq.o \
./Middlewares/STM32_USB_Device_Library/Core/usbd_ioreq.o 

C_DEPS += \
./Middlewares/STM32_USB_Device_Library/Core/usbd_core.d \
./Middlewares/STM32_USB_Device_Library/Core/usbd_ctlreq.d \
./Middlewares/STM32_USB_Device_Library/Core/usbd_ioreq.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/STM32_USB_Device_Library/Core/usbd_core.o: C:/Users/gibutakov/STM32CubeIDE/workspace_1.16.1/Zigbee_OnOff_Server_Coord/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c Middlewares/STM32_USB_Device_Library/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_STM32WBXX_USB_DONGLE -DZIGBEE_WB -DDEBUG -DSTM32WB55xx -c -I../../Core/Inc -I../../STM32_WPAN/App -I../../Drivers/BSP/P-NUCLEO-WB55.USBDongle -I../../Drivers/STM32WBxx_HAL_Driver/Inc -I../../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_WPAN -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../../Middlewares/ST/STM32_WPAN/utilities -I../../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../Middlewares/ST/STM32_WPAN/zigbee/core/inc -I../../Middlewares/ST/STM32_WPAN/zigbee/stack -I../../Middlewares/ST/STM32_WPAN/zigbee/stack/include -I../../Middlewares/ST/STM32_WPAN/zigbee/stack/include/mac -I../../Drivers/CMSIS/Include -I../../Utilities/sequencer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/vcp -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_USB_Device_Library/Core/usbd_ctlreq.o: C:/Users/gibutakov/STM32CubeIDE/workspace_1.16.1/Zigbee_OnOff_Server_Coord/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c Middlewares/STM32_USB_Device_Library/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_STM32WBXX_USB_DONGLE -DZIGBEE_WB -DDEBUG -DSTM32WB55xx -c -I../../Core/Inc -I../../STM32_WPAN/App -I../../Drivers/BSP/P-NUCLEO-WB55.USBDongle -I../../Drivers/STM32WBxx_HAL_Driver/Inc -I../../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_WPAN -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../../Middlewares/ST/STM32_WPAN/utilities -I../../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../Middlewares/ST/STM32_WPAN/zigbee/core/inc -I../../Middlewares/ST/STM32_WPAN/zigbee/stack -I../../Middlewares/ST/STM32_WPAN/zigbee/stack/include -I../../Middlewares/ST/STM32_WPAN/zigbee/stack/include/mac -I../../Drivers/CMSIS/Include -I../../Utilities/sequencer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/vcp -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Middlewares/STM32_USB_Device_Library/Core/usbd_ioreq.o: C:/Users/gibutakov/STM32CubeIDE/workspace_1.16.1/Zigbee_OnOff_Server_Coord/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c Middlewares/STM32_USB_Device_Library/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DUSE_STM32WBXX_USB_DONGLE -DZIGBEE_WB -DDEBUG -DSTM32WB55xx -c -I../../Core/Inc -I../../STM32_WPAN/App -I../../Drivers/BSP/P-NUCLEO-WB55.USBDongle -I../../Drivers/STM32WBxx_HAL_Driver/Inc -I../../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../../Middlewares/ST/STM32_WPAN -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../../Middlewares/ST/STM32_WPAN/utilities -I../../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../../Middlewares/ST/STM32_WPAN/zigbee/core/inc -I../../Middlewares/ST/STM32_WPAN/zigbee/stack -I../../Middlewares/ST/STM32_WPAN/zigbee/stack/include -I../../Middlewares/ST/STM32_WPAN/zigbee/stack/include/mac -I../../Drivers/CMSIS/Include -I../../Utilities/sequencer -I../../Utilities/lpm/tiny_lpm -I../../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/vcp -I../../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-STM32_USB_Device_Library-2f-Core

clean-Middlewares-2f-STM32_USB_Device_Library-2f-Core:
	-$(RM) ./Middlewares/STM32_USB_Device_Library/Core/usbd_core.cyclo ./Middlewares/STM32_USB_Device_Library/Core/usbd_core.d ./Middlewares/STM32_USB_Device_Library/Core/usbd_core.o ./Middlewares/STM32_USB_Device_Library/Core/usbd_core.su ./Middlewares/STM32_USB_Device_Library/Core/usbd_ctlreq.cyclo ./Middlewares/STM32_USB_Device_Library/Core/usbd_ctlreq.d ./Middlewares/STM32_USB_Device_Library/Core/usbd_ctlreq.o ./Middlewares/STM32_USB_Device_Library/Core/usbd_ctlreq.su ./Middlewares/STM32_USB_Device_Library/Core/usbd_ioreq.cyclo ./Middlewares/STM32_USB_Device_Library/Core/usbd_ioreq.d ./Middlewares/STM32_USB_Device_Library/Core/usbd_ioreq.o ./Middlewares/STM32_USB_Device_Library/Core/usbd_ioreq.su

.PHONY: clean-Middlewares-2f-STM32_USB_Device_Library-2f-Core

