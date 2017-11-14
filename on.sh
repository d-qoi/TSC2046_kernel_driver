#!/bin/bash
# From: https://gist.github.com/jadonk/0e4a190fc01dc5723d1f183737af1d83

# Connect the display before running this.

export GPIO=/sys/class/gpio
export OCP=/sys/devices/platform/ocp
export CS=7        # CS - C18, Chip Select for SPI1.2, GPIO0_7
export IRQ=98	    # GPIO Interrupt

sudo bash << EOF
    # Remove the modules
    if lsmod | grep -q 'TSC2046_driver'; then rmmod TSC2046_driver; 	fi

    # Set the pinmuxes for the display
    # echo gpio > $OCP/ocp\:J15_pinmux/state   # IRQ  - J15 - GP1_3 , GPIO3_2
    echo spi  > $OCP/ocp\:P9_31_pinmux/state  # SCLK - A13 - S1.1_5
    echo spi  > $OCP/ocp\:P9_29_pinmux/state  # MISO - B13 - S1.1_4
    echo spi  > $OCP/ocp\:P9_30_pinmux/state  # MOSI - D12 - S1.1_3
    
    # Set chip select  C18
    if [ -d $GPIO/gpio$CS ]; then echo $CS    > $GPIO/unexport; fi
    echo spi > $OCP/ocp\:C18_pinmux/state # CS - C18 - S1.2_6
    
    sleep 1
    #modprobe TSC2046_driver penirq=98 spics=1 spiport=1
    sudo insmod TSC2046_driver.ko penirq=98 spics=1 spiport=1 SER=0 mode=0
EOF
