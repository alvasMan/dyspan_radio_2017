#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --freq=3.195e9 --tx_enabled=false --sensing=true --learning=false --sensing_to_file=false --txgain_soft=-12 --rxgain=10 --challenge=true --debug=false --rxsubdev="A:0" --txsubdev="A:0" --args="master_clock_rate=120e6" --deep_learning=true --rxantenna="RX2" --db_ip="192.168.52.1" --phase_num=1 --new_db=true

#"192.168.5.221"
#"192.168.52.1
