#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
#./dyspanradio --freq=2.49e9 --channel_bandwidth=5000000 --channel_rate=5000000 --rxgain=25 --num_channels=4 --txgain_uhd=5 --txgain_soft=-12  --rxsubdev="A:0" --txsubdev="A:0"
./dyspanradio --freq=2.49e9 --channel_bandwidth=5000000 --channel_rate 5000000 --num_channels=4 --txgain_uhd=15 --txgain_soft=-12 --rxgain=31 --challenge=false --debug=false --rxsubdev="A:0" --txsubdev="A:0" --mod=qam16 --crc crc16
