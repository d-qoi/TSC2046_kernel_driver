# TSC2046_kernel_driver
Driver for the TSC2046 Touchscreen controller.

* Built for the BeagleBone Blue

Check out [This article here](https://elinux.org/ECE497_Project:_TSC2046_Kernel_Driver) for documentation.

## Installation
1. Install Pre-reqs
 * linux-headers-`uname –r`
2. Make this project
 * cd to project directory and type `make`
3. Turn it on
 * While in the project dir type `./on.sh`
4. To turn in off
 * While in the project dir type `./off.sh`

### What are `on.sh` and `off.sh`
These two files configure the beaglebone's pinmux to let the SPI1 bus be used as an SPI bus.

### Default pins
SPI1.2 port
GP1\_3 pin
