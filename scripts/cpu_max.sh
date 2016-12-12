#!/bin/bash
for f in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor ; do
 echo performance > $f
done
