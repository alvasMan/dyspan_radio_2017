#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
#./dyspanradio --freq=2.49e9 --channel_bandwidth=5000000 --channel_rate=5000000 --rxgain=25 --num_channels=4 --txgain_uhd=5 --txgain_soft=-12  --rxsubdev="A:0" --txsubdev="A:0"
./dyspanradio --freq=3.5e9 --channel_bandwidth=2500000 --channel_rate 2500000 --num_channels=4 --txgain_uhd=20 --rxgain=31 --sensing=false --power_control=true --channel_hopping=false --challenge=true --debug=false --rxsubdev="A:0" --txsubdev="A:0" --crc crc16 --db_period=500 --db_ip="192.168.5.221"
