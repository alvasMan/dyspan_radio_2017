#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --args="master_clock_rate=40e6" --freq=2412500000 --channel_bandwidth=5000000 --channel_rate=5000000 --rxgain=50 --num_channels=4 --txgain_uhd=83 --txgain_soft=-24
