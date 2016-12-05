#!/bin/sh

dfu-util -a 0 --dfuse-address 0x08010000 -D /tmp/ArduCopter.build/revomini_MP32V1F4.bin

