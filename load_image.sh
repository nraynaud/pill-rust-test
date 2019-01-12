#!/usr/bin/env bash

killall openocd
openocd -f openocd.cfg -c "flash_image $1"