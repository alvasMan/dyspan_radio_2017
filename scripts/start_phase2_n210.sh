#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --freq=3.195e9 --num_channels=4 --txgain_soft=-12 --rxgain=31 --challenge=true --debug=false --rxsubdev="A:0" --txsubdev="A:0" --mod=qam16 --crc crc16 --sensing=true --db_ip="192.168.52.1" --rxantenna="RX2" --channel_hopping=true --power_control=true --db_period=500 --txgain_uhd=5 --max_gain=30 --min_gain=1 --stop_gain_sensing=25 --phase_num=1 --new_db=true
