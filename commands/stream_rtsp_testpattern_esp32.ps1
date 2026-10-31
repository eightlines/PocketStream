# Alternate ffmpeg script for ESP32 H.264 decoder compatibility
# - Remove -tune zerolatency
# - Use -preset veryfast
# - Lower bitrate for easier decoding
# - Keep Baseline profile, keyint=1

ffmpeg -re -f lavfi -i testsrc=size=320x240:rate=10 -c:v libx264 -g 1 -profile:v baseline -pix_fmt yuv420p -x264-params keyint=1:min-keyint=1:scenecut=0 -preset veryfast -b:v 500k -f rtsp rtsp://127.0.0.1:8554/live
