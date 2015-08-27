#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --mode=rx --freq=2.49e9 --channel_bandwidth=10000000 --rxgain=25 --num_channels=2 --args="recv_frame_size=4000" --subdev="A:0 A:0"
