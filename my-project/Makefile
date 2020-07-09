PROJECT = usbhs
BUILD_DIR = bin

ARCH_FLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard
CFLAGS = -DUSE_HAL_DRIVER -DSTM32F723xx
INCLUDES = -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include


SHARED_DIR = ../my-common-code

XOBJS = build/usb_device.o
XOBJS += build/usbd_conf.o
XOBJS += build/usbd_desc.o
XOBJS += build/usbd_cdc_if.o
build/usb_device.o: Src/usb_device.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/usb_device.d" -Wa,-a,-ad,-alms=build/usb_device.lst Src/usb_device.c -o build/usb_device.o
build/usbd_conf.o: Src/usbd_conf.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/usbd_conf.d" -Wa,-a,-ad,-alms=build/usbd_conf.lst Src/usbd_conf.c -o build/usbd_conf.o
build/usbd_desc.o: Src/usbd_desc.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/usbd_desc.d" -Wa,-a,-ad,-alms=build/usbd_desc.lst Src/usbd_desc.c -o build/usbd_desc.o
build/usbd_cdc_if.o: Src/usbd_cdc_if.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/usbd_cdc_if.d" -Wa,-a,-ad,-alms=build/usbd_cdc_if.lst Src/usbd_cdc_if.c -o build/usbd_cdc_if.o
build/stm32f7xx_it.o: Src/stm32f7xx_it.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/stm32f7xx_it.d" -Wa,-a,-ad,-alms=build/stm32f7xx_it.lst Src/stm32f7xx_it.c -o build/stm32f7xx_it.o
build/stm32f7xx_hal_msp.o: Src/stm32f7xx_hal_msp.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/stm32f7xx_hal_msp.d" -Wa,-a,-ad,-alms=build/stm32f7xx_hal_msp.lst Src/stm32f7xx_hal_msp.c -o build/stm32f7xx_hal_msp.o


XOBJS += build/stm32f7xx_hal_pcd.o
build/stm32f7xx_hal_pcd.o: Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/stm32f7xx_hal_pcd.d" -Wa,-a,-ad,-alms=build/stm32f7xx_hal_pcd.lst Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c -o build/stm32f7xx_hal_pcd.o
XOBJS += build/stm32f7xx_hal_pcd_ex.o
build/stm32f7xx_hal_pcd_ex.o: Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/stm32f7xx_hal_pcd_ex.d" -Wa,-a,-ad,-alms=build/stm32f7xx_hal_pcd_ex.lst Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c -o build/stm32f7xx_hal_pcd_ex.o
XOBJS += build/stm32f7xx_ll_usb.o
build/stm32f7xx_ll_usb.o: Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/stm32f7xx_ll_usb.d" -Wa,-a,-ad,-alms=build/stm32f7xx_ll_usb.lst Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c -o build/stm32f7xx_ll_usb.o
XOBJS += build/stm32f7xx_hal_rcc.o
build/stm32f7xx_hal_rcc.o: Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/stm32f7xx_hal_rcc.d" -Wa,-a,-ad,-alms=build/stm32f7xx_hal_rcc.lst Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c -o build/stm32f7xx_hal_rcc.o
XOBJS += build/stm32f7xx_hal_rcc_ex.o
build/stm32f7xx_hal_rcc_ex.o: Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/stm32f7xx_hal_rcc_ex.d" -Wa,-a,-ad,-alms=build/stm32f7xx_hal_rcc_ex.lst Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c -o build/stm32f7xx_hal_rcc_ex.o
XOBJS += build/stm32f7xx_hal_pwr_ex.o
build/stm32f7xx_hal_pwr_ex.o: Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/stm32f7xx_hal_pwr_ex.d" -Wa,-a,-ad,-alms=build/stm32f7xx_hal_pwr_ex.lst Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c -o build/stm32f7xx_hal_pwr_ex.o
XOBJS += build/stm32f7xx_hal_cortex.o
build/stm32f7xx_hal_cortex.o: Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/stm32f7xx_hal_cortex.d" -Wa,-a,-ad,-alms=build/stm32f7xx_hal_cortex.lst Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c -o build/stm32f7xx_hal_cortex.o
XOBJS += build/stm32f7xx_hal.o
build/stm32f7xx_hal.o: Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/stm32f7xx_hal.d" -Wa,-a,-ad,-alms=build/stm32f7xx_hal.lst Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c -o build/stm32f7xx_hal.o


XOBJS += build/system_stm32f7xx.o
build/system_stm32f7xx.o: Src/system_stm32f7xx.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/system_stm32f7xx.d" -Wa,-a,-ad,-alms=build/system_stm32f7xx.lst Src/system_stm32f7xx.c -o build/system_stm32f7xx.o
XOBJS += build/main.o
build/main.o: Src/main.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/main.d" -Wa,-a,-ad,-alms=build/main.lst Src/main.c -o build/main.o


XOBJS += build/usbd_core.o
build/usbd_core.o: Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/usbd_core.d" -Wa,-a,-ad,-alms=build/usbd_core.lst Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c -o build/usbd_core.o
XOBJS += build/usbd_ctlreq.o
build/usbd_ctlreq.o: Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/usbd_ctlreq.d" -Wa,-a,-ad,-alms=build/usbd_ctlreq.lst Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c -o build/usbd_ctlreq.o
XOBJS += build/usbd_ioreq.o
build/usbd_ioreq.o: Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/usbd_ioreq.d" -Wa,-a,-ad,-alms=build/usbd_ioreq.lst Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c -o build/usbd_ioreq.o
XOBJS += build/usbd_cdc.o
build/usbd_cdc.o: Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
	arm-none-eabi-gcc -c -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -DUSE_HAL_DRIVER -DSTM32F723xx -IInc -IDrivers/STM32F7xx_HAL_Driver/Inc -IDrivers/STM32F7xx_HAL_Driver/Inc/Legacy -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -IDrivers/CMSIS/Device/ST/STM32F7xx/Include -IDrivers/CMSIS/Include -IDrivers/CMSIS/Include -Og -Wall -fdata-sections -ffunction-sections -g -gdwarf-2 -MMD -MP -MF"build/usbd_cdc.d" -Wa,-a,-ad,-alms=build/usbd_cdc.lst Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c -o build/usbd_cdc.o

CFILES += my-project.c
CFILES += cdcacm.c


# CFILES = Src/main.c
# CFILES += Src/usb_device.c
# CFILES += Src/usbd_conf.c
# CFILES += Src/usbd_desc.c
# CFILES += Src/usbd_cdc_if.c
# CFILES += Src/stm32f7xx_it.c
# CFILES += Src/stm32f7xx_hal_msp.c

# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pcd_ex.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_ll_usb.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_i2c_ex.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_exti.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c
# CFILES += Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c

# CFILES += Src/system_stm32f7xx.c

# CFILES += Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
# CFILES += Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
# CFILES += Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
# CFILES += Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c

DEVICE=stm32f723iek6

# You shouldn't have to edit anything below here.
VPATH += $(SHARED_DIR)
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))
OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
