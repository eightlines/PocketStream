# Webcam stream for ESP32-P4
# Resolution must match VIDEO_WIDTH/HEIGHT in main.c

ffmpeg.exe `
    -f dshow -i video="Opal Tadpole" `
    -vf "scale=240:192" `
    -c:v h264_mf `
    -b:v 100k `
    -g 1 `
    -pix_fmt nv12 `
    -f h264 `
    "udp://10.0.0.2:3333?pkt_size=1200"