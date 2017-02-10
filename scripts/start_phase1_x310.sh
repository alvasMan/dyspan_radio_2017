#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --freq=1.255e9 --tx_enabled=false --sensing=true --learning=false --sensing_to_file=false --txgain_uhd=35 --txgain_soft=-12 --rxgain=20 --challenge=false --debug=false --rxsubdev="A:0" --txsubdev="B:0" --mod=qpsk --crc crc16 --args="master_clock_rate=120e6"
