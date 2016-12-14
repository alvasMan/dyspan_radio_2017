#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --mode=rx --freq=2.49e9 --channel_bandwidth=2500000 --rxgain=25 --num_channels=4 --args="addr0=192.168.10.2,addr1=192.168.10.3,recv_frame_size=4000" --rxsubdev="A:0 A:0" --challenge=false --db_ip=192.168.2.101 --debug=false --numtrx 2 --rxantenna J1 --channel_rate=2500000
