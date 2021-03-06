#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --freq=3.195e9 --tx_enabled=false --sensing=true --learning=true --sensing_to_file=true --txgain_soft=-12 --rxgain=10 --challenge=false --debug=false --rxsubdev="A:0" --txsubdev="A:0" --args="master_clock_rate=120e6" --rxantenna "RX2"
