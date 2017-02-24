#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --freq=3.195e9 --channel_bandwidth=2500000 --channel_rate 2500000 --num_channels=4 --txgain_uhd=30 --txgain_soft=-12 --rxgain=31 --challenge=true --debug=false --rxsubdev="A:0" --txsubdev="A:0" --mod=qam16 --crc crc16 --db_ip="192.168.5.221" --rxantenna="RX2"
