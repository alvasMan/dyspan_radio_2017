#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
RUN_CMD="./dyspanradio --freq=3.195e9 --txgain_uhd=0 --rxgain=31 --debug=false --rxsubdev=A:0 --txsubdev=A:0 --crc crc16 --fec1=h128 --calibration=true --challenge=true --db_period=400 --db_ip=192.168.52.1 --sensing=false --new_db=true --phase_num=1"

#valgrind --tool=memcheck $RUN_CMD
#gdb --args $RUN_CMD
$RUN_CMD
