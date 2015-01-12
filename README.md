# grbl_stm32f4

## Purpose

This a port of the TinyG g-code interpreter to the STM32F4 SoC

The STM32F4 SoC has attractive features for a realtime G-Code interpreter and machine controller.

* Lot's of IO - it's possible to run more stepper motors, more limit switches, etc.
* A Floating Point Unit - good for doing the linear/circular interpolation math.
* Fast (168MHz 32-bit).

## Target Platform

STM32F4Discovery Board
* http://www.st.com/web/catalog/tools/FM116/SC959/SS1532/PF252419
* http://www.digikey.com/short/504hf

## Build tools:

GCC Toolchain
* https://launchpad.net/gcc-arm-embedded
* [gcc-arm-none-eabi-4_8-2014q2-20140609] (https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q2-update/+download/gcc-arm-none-eabi-4_8-2014q2-20140609-linux.tar.bz2)

STLink Programming Tool
* https://github.com/texane/stlink

## Status

* Currently Non-Functional
 

