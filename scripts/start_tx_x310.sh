#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --freq=2.49e9 --txgain_uhd=35 --txgain_soft=-12 --rxgain=31 --challenge=false --sensing=false --debug=false --rxsubdev="B:0" --txsubdev="B:0" --mod=qpsk --crc crc16 --args="master_clock_rate=120e6"
