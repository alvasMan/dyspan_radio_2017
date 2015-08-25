#!/bin/sh
BUILD_DIR=../build_release/src
cd $BUILD_DIR
./dyspanradio --tx=false --freq=2.495e9 --channel_bandwidth=5000000 --rxgain=25 --num_channels=4 --args="recv_frame_size=4000"
