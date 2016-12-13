#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --freq=2.49e9 --channel_bandwidth=5000000 --channel_rate 5000000 --num_channels=4 --txgain_uhd=35 --txgain_soft=-12 --rxgain=31 --challenge=false --debug=false --rxsubdev="B:0" --txsubdev="A:0" --mod=qpsk --crc crc16 --args="master_clock_rate=120e6"
