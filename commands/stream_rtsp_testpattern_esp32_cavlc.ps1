# ESP32 H.264 decoder compatibility test: force CAVLC, no B-frames, low level
# - Explicitly disable CABAC (use CAVLC)
# - Disable B-frames
# - Set H.264 level to 3.0
# - Baseline profile, keyint=1, yuv420p

ffmpeg -re -f lavfi -i testsrc=size=320x240:rate=10 -c:v libx264 -g 1 -profile:v baseline -pix_fmt yuv420p -x264-params keyint=1:min-keyint=1:scenecut=0:cabac=0 -bf 0 -level 3.0 -preset veryfast -b:v 500k -f rtsp rtsp://127.0.0.1:8554/live
