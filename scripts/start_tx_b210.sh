#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --freq=2.49e9 --num_channels=4 --txgain_uhd=70 --txgain_soft=-12 --rxgain=1 --challenge=false --debug=false --rxsubdev="A:B" --txsubdev="A:A" --mod=qpsk --crc crc16 --sensing=false
