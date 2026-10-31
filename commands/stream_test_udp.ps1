# H.264 UDP streaming - Test pattern
# Testing higher resolution

$ESP32_IP = "10.0.0.2"

Write-Host "Streaming 240x192 H.264 test pattern to ESP32" -ForegroundColor Cyan
Write-Host "Press q to stop" -ForegroundColor Yellow

# Generate test pattern
# - 240x192 resolution (4x pixels vs 160x128)
# - Higher bitrate for larger frames
# - Every frame is keyframe for UDP resilience
ffmpeg -f lavfi -i "testsrc=size=240x192:rate=30" `
    -pix_fmt nv12 `
    -c:v h264_mf `
    -b:v 300k `
    -g 1 `
    -f h264 "udp://${ESP32_IP}:3333?pkt_size=1400"
