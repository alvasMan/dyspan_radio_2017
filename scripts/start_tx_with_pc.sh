#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
#./dyspanradio --freq=2.49e9 --channel_bandwidth=5000000 --channel_rate=5000000 --rxgain=25 --num_channels=4 --txgain_uhd=5 --txgain_soft=-12  --rxsubdev="A:0" --txsubdev="A:0"
./dyspanradio --freq=3.195e9 --channel_bandwidth=2500000 --channel_rate 2500000 --num_channels=4 --txgain_uhd=20 --rxgain=31 --challenge=true --debug=false --rxsubdev="A:0" --txsubdev="A:0" --crc crc16 --db_period=400 --db_ip="192.168.5.221" --deep-learning=false
