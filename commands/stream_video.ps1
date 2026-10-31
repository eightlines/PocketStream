# H264 stream for ESP32-P4 TinyH264 decoder using Windows MediaFoundation
# 160x128 resolution at 30fps - scaled to 720x720 on device

ffmpeg.exe -re `
    -f lavfi -i "smptebars=size=160x128:rate=30" `
    -c:v h264_mf `
    -b:v 200k `
    -g 30 `
    -pix_fmt nv12 `
    -f h264 `
    "udp://10.0.0.2:3333?pkt_size=1400"