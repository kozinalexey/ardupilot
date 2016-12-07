#!/bin/sh

#production binary for bootloader
#dfu-util -a 0 --dfuse-address 0x08010000 -D /tmp/ArduCopter.build/revomini_MP32V1F4.bin

# bare metal binary
dfu-util -a 0 --dfuse-address 0x08000000 -D /tmp/ArduCopter.build/revomini_MP32V1F4.bin

