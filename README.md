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
