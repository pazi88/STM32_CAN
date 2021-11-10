# STM32_CAN

This is universal CAN library for STM32 that was made to be used with Speeduino EFI and in my other projects.
It should support all STM32 MCUs that are also supported in stm32duino Arduino_Core_STM32 and supports up to 3x CAN busses.
The library is created because at least currently (year 2021) there is no official CAN library in the STM32 core.
This library is based on several STM32 CAN example libraries linked below and it has been combined with few
things from Teensy FlexCAN library to make it compatible with the CAN features that exist in speeduino for Teensy.
Links to repositories that have helped with this:
https://github.com/nopnop2002/Arduino-STM32-CAN
https://github.com/J-f-Jensen/libraries/tree/master/STM32_CAN
https://github.com/jiauka/STM32F1_CAN
STM32 core: https://github.com/stm32duino/Arduino_Core_STM32
