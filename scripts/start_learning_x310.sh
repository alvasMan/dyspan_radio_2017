#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --freq=1.255e9 --tx_enabled=false --learning=true --sensing_to_file=true --txgain_uhd=20 --txgain_soft=-12 --rxgain=31 --challenge=false --debug=false --rxsubdev="B:0" --txsubdev="B:0" --args="master_clock_rate=120e6"
