#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
RUN_CMD="./dyspanradio --freq=2.49e9 --channel_bandwidth=2500000 --channel_rate 2500000 --num_channels=4 --txgain_soft=-12 --rxgain=31 --debug=false --rxsubdev=A:B --txsubdev=A:B --crc crc16 --change_mod_period=100 --calibration=true --challenge=true --db_ip=192.168.5.221"

#valgrind --tool=memcheck $RUN_CMD
#gdb --args $RUN_CMD
$RUN_CMD
