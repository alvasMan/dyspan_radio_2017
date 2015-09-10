#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --freq=2.49e9 --channel_bandwidth=5000000 --channel_rate=5000000 --rxgain=25 --num_channels=4 --txgain_uhd=5 --txgain_soft=-12  --rxsubdev="A:0" --txsubdev="A:0"
