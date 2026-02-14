# ESP32 H.264 decoder compatibility test: 720x720, CAVLC, no B-frames, low level
# - Output 720x720 test pattern
# - Explicitly disable CABAC (use CAVLC)
# - Disable B-frames
# - Set H.264 level to 3.0
# - Baseline profile, keyint=1, yuv420p

ffmpeg -re -f lavfi -i testsrc=size=720x720:rate=15 -r 15 -c:v libx264 -g 1 -profile:v baseline -pix_fmt yuv420p -x264-params keyint=1:min-keyint=1:scenecut=0:cabac=0 -bf 0 -level 3.0 -preset veryfast -b:v 2000k -f rtsp rtsp://127.0.0.1:8554/live
