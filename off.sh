#!/bin/sh
# From: https://gist.github.com/jadonk/0e4a190fc01dc5723d1f183737af1d83

# Connect the display before running this.

export GPIO=/sys/class/gpio
export OCP=/sys/devices/platform/ocp
export CS=7        # CS - C18, Chip Select for SPI1.2, GPIO0_7
export IRQ=98	    # GPIO Interrupt

sudo bash << EOF
    # remove the framebuffer modules
    if lsmod | grep -q 'TSC2046_driver ' ; then rmmod TSC2046_driver;  fi
EOF
