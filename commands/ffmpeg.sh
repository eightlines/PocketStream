#!/bin/sh
# Using testsrc (test pattern) for validation.
# Reduced FPS to 15 and Bitrate to 500k for stability
ffmpeg -re -f lavfi -i testsrc=size=320x320:rate=15 -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p -b:v 500k -g 30 -f h264 "udp://192.168.137.70:3333?pkt_size=1316"