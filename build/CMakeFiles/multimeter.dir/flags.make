# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# compile ASM with /usr/bin/arm-none-eabi-gcc
# compile C with /usr/bin/arm-none-eabi-gcc
# compile CXX with /usr/bin/arm-none-eabi-g++
ASM_FLAGS = -g   -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Wformat=2 -Wformat-truncation -Wundef -fno-common -Wno-unused-parameter -O0 -g3 -ggdb

ASM_DEFINES = -DSTM32F411xE -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER

ASM_INCLUDES = -I/home/valentin/robotics/MultiMeter/Project/Inc -I/home/valentin/robotics/MultiMeter/Project/Inc/ADXL345 -I/home/valentin/robotics/MultiMeter/Project/Inc/BMP085 -I/home/valentin/robotics/MultiMeter/Project/Inc/HMC5883L -I/home/valentin/robotics/MultiMeter/Project/Inc/ITG3200 -I/home/valentin/robotics/MultiMeter/Project/Inc/MPU6050 -I/home/valentin/robotics/MultiMeter/Project/Inc/VL53L5CX -I/home/valentin/robotics/MultiMeter/Project/Inc/QMC5883L -isystem /home/valentin/robotics/MultiMeter/Core/Inc -isystem /home/valentin/robotics/MultiMeter/Drivers/STM32F4xx_HAL_Driver/Inc -isystem /home/valentin/robotics/MultiMeter/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -isystem /home/valentin/robotics/MultiMeter/Drivers/CMSIS/Device/ST/STM32F4xx/Include -isystem /home/valentin/robotics/MultiMeter/Drivers/CMSIS/Include -isystem /home/valentin/robotics/MultiMeter/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -isystem /home/valentin/robotics/MultiMeter/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -isystem /home/valentin/robotics/MultiMeter/USB_DEVICE/App -isystem /home/valentin/robotics/MultiMeter/USB_DEVICE/Target 

C_FLAGS = -fdata-sections -ffunction-sections --specs=nano.specs -Wl,--gc-sections -g   -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Wformat=2 -Wformat-truncation -Wundef -fno-common -Wno-unused-parameter -O0 -g3 -ggdb -std=gnu11

C_DEFINES = -DSTM32F411xE -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER

C_INCLUDES = -I/home/valentin/robotics/MultiMeter/Project/Inc -I/home/valentin/robotics/MultiMeter/Project/Inc/ADXL345 -I/home/valentin/robotics/MultiMeter/Project/Inc/BMP085 -I/home/valentin/robotics/MultiMeter/Project/Inc/HMC5883L -I/home/valentin/robotics/MultiMeter/Project/Inc/ITG3200 -I/home/valentin/robotics/MultiMeter/Project/Inc/MPU6050 -I/home/valentin/robotics/MultiMeter/Project/Inc/VL53L5CX -I/home/valentin/robotics/MultiMeter/Project/Inc/QMC5883L -isystem /home/valentin/robotics/MultiMeter/Core/Inc -isystem /home/valentin/robotics/MultiMeter/Drivers/STM32F4xx_HAL_Driver/Inc -isystem /home/valentin/robotics/MultiMeter/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -isystem /home/valentin/robotics/MultiMeter/Drivers/CMSIS/Device/ST/STM32F4xx/Include -isystem /home/valentin/robotics/MultiMeter/Drivers/CMSIS/Include -isystem /home/valentin/robotics/MultiMeter/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -isystem /home/valentin/robotics/MultiMeter/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -isystem /home/valentin/robotics/MultiMeter/USB_DEVICE/App -isystem /home/valentin/robotics/MultiMeter/USB_DEVICE/Target 

CXX_FLAGS = -fdata-sections -ffunction-sections --specs=nano.specs -Wl,--gc-sections -fno-rtti -fno-exceptions -fno-threadsafe-statics -g   -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wall -Wextra -Wpedantic -Wshadow -Wdouble-promotion -Wformat=2 -Wformat-truncation -Wundef -fno-common -Wno-unused-parameter -Wconversion -Wno-volatile -Wold-style-cast -Wuseless-cast -Wsuggest-override -O0 -g3 -ggdb -std=gnu++2a

CXX_DEFINES = -DSTM32F411xE -DUSE_FULL_LL_DRIVER -DUSE_HAL_DRIVER

CXX_INCLUDES = -I/home/valentin/robotics/MultiMeter/Project/Inc -I/home/valentin/robotics/MultiMeter/Project/Inc/ADXL345 -I/home/valentin/robotics/MultiMeter/Project/Inc/BMP085 -I/home/valentin/robotics/MultiMeter/Project/Inc/HMC5883L -I/home/valentin/robotics/MultiMeter/Project/Inc/ITG3200 -I/home/valentin/robotics/MultiMeter/Project/Inc/MPU6050 -I/home/valentin/robotics/MultiMeter/Project/Inc/VL53L5CX -I/home/valentin/robotics/MultiMeter/Project/Inc/QMC5883L -isystem /home/valentin/robotics/MultiMeter/Core/Inc -isystem /home/valentin/robotics/MultiMeter/Drivers/STM32F4xx_HAL_Driver/Inc -isystem /home/valentin/robotics/MultiMeter/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -isystem /home/valentin/robotics/MultiMeter/Drivers/CMSIS/Device/ST/STM32F4xx/Include -isystem /home/valentin/robotics/MultiMeter/Drivers/CMSIS/Include -isystem /home/valentin/robotics/MultiMeter/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -isystem /home/valentin/robotics/MultiMeter/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -isystem /home/valentin/robotics/MultiMeter/USB_DEVICE/App -isystem /home/valentin/robotics/MultiMeter/USB_DEVICE/Target -isystem /usr/include/newlib/nano -isystem /usr/lib/arm-none-eabi/include/c++/9.2.1 -isystem /usr/lib/arm-none-eabi/include/c++/9.2.1/arm-none-eabi -isystem /usr/lib/arm-none-eabi/include/c++/9.2.1/backward -isystem /usr/lib/gcc/arm-none-eabi/9.2.1/include -isystem /usr/lib/gcc/arm-none-eabi/9.2.1/include-fixed -isystem /usr/lib/arm-none-eabi/include 

