#! /bin/bash

while true; do
	clear
	echo "Values"
	cat /sys/firmware/TSC2046/device/vals
	echo "Diffs"
	cat /sys/firmware/TSC2046/device/diffs
	sleep 0.3
done
