# ESP32 H.264 decoder compatibility test: 32x32, CAVLC, no B-frames, low level
# - Output 32x32 test pattern
# - Explicitly disable CABAC (use CAVLC)
# - Disable B-frames
# - Set H.264 level to 3.0
# - Baseline profile, keyint=1, yuv420p

$RTSP_URL = "rtsp://127.0.0.1:8554/live"  # Set your MediaMTX RTSP URL

ffmpeg -re -f lavfi -i testsrc=size=512x512:rate=30 -r 30 `
    -c:v libx264 -profile:v baseline -level 3.0 -pix_fmt yuv420p `
    -x264-params keyint=15:min-keyint=15:scenecut=0:repeat-headers=1:cabac=0 `
    -bf 0 -preset veryfast -b:v 2000k `
    -f rtsp $RTSP_URL