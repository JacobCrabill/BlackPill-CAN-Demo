# BlackPill CAN Demo

Demo of CAN communications on a RobotDyn Black Pill board (STM32F103C8T6)

# Goals

The impetus for this project actually started out with much loftier goals: to develop a general-purpose CAN-to-CAN protocol converter on top of NuttX / PX4 using a cheap STM32 microcontroller.  Turns out there's a lot of details involved.

This particular project here is a sub-goal of that: to learn microcontroller programming on STM MCUs using their development framework (STM32Cube).  I thought it would be easy to find some example online for general-purpose CAN communication for any STM32 board, but I have come to realize that's not the case.  Most examples either just show you how to blink an LED, or if they get into more complex / interesting use cases, they're for some specific development board that I was too cheap to purchase.  So, here we go: a tutorial solving my initial sub-goal -- cheap & simple CAN communication -- on dirt-cheap hardware you can buy on Amazon.

# Hardware

The hardware list for a complete working version of this demo is as follows:

1. RobotDyn Black Pill board, x2
    + With pre-soldered headers or no, take your pick based on your willingness to solder headers onto a PCB)
2. CAN transceiver, x2
    + I bought the [Wave Share](https://www.amazon.com/waveshare-SN65HVD230-Transceiver-Communication-Development/dp/B076NRGCKY/) boards for ease of use
3. ST-Link v2 clone, xx1
    + Many options on [Amazon](https://www.amazon.com/HiLetgo-Emulator-Downloader-Programmer-STM32F103C8T6/dp/B07SQV6VLZ/)
4. Mico-USB cable
5. Breadboard / jumper wires

# Using the Demo

## Building the Code

The project uses standard GNU Makefiles.  Any recent version of the GNU toolchain for Arm architectures should work; I have been using the 2017 q4 release:

```
$ arm-none-eabi-gcc --version
arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
Copyright (C) 2017 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE
```

Assuming the compilers are in your PATH variable, just type "make".

## Loading the Firmware

My preferred method has been to load the binary file directly into the flash of the target using an ST-Link v2 dongle.  Open-source versions of the stlink utilities for Linux can be found (here)[https://github.com/texane/stlink].

Once you have the stlink executables, hook up the ST-Link v2 programmer to the Black Pill board (match GND to GND, CLK to CLK, etc.), set the BOOT0 jumper to "1" and the BOOT1 jumper to "0" (this puts the board into firmware-load mode once powered on).  The BOOT0 jumper is the one closer to the USB connector.  Plug the ST-Link into your computer then do:

``` st-flash write build/BlackPill-CAN.bin 0x08000000 ```

Occasionally you may need to erase before programming new firmware:

``` st-flash erase ```

Once you're done programming, you can set the BOOT0 jumper back to "0" to load the new program on boot instead of staying in firmware-load mode.  You'll then also be able to push the "RST" button to reboot to board while it's connected to a power supply.

### Happy coding!
