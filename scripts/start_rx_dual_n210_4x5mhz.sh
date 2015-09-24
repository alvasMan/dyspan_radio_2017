#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
#./dyspanradio --mode=rx --freq=2.49e9 --channel_bandwidth=5000000 --rxgain=25 --num_channels=4 --args="addr0=192.168.70.2,addr1=192.168.70.3,recv_frame_size=4000" --subdev="A:0 A:0"
./dyspanradio --mode=rx --freq=2.49e9 --channel_bandwidth=5000000 --rxgain=25 --num_channels=4 --args="addr0=192.168.70.2,addr1=192.168.70.3,recv_frame_size=4000" --rxsubdev="A:0 A:0" --challenge=false --db_ip=192.168.1.104 --debug=true
