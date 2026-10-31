# RTSP to ESP32 bridge
# Receives RTSP stream and re-encodes for ESP32

$RTSP_URL = "rtsp://10.0.0.1:8554/live"  # TouchDesigner RTSP
$ESP32_IP = "10.0.0.2"

# Video resolution - start small for reliability, increase if stable
$WIDTH = 240
$HEIGHT = 240

Write-Host "Bridging RTSP to ESP32 at ${ESP32_IP}:3333" -ForegroundColor Cyan
Write-Host "RTSP source: $RTSP_URL" -ForegroundColor Yellow
Write-Host "Output: ${WIDTH}x${HEIGHT} H.264 over UDP" -ForegroundColor Yellow
Write-Host "Press q to stop" -ForegroundColor Yellow

# Receive RTSP, re-encode for ESP32
# - Square crop to match display aspect
# - Low bitrate to fit in UDP packets without fragmentation
# - Every frame is keyframe (g=1) to handle packet loss
# - Low latency settings
ffmpeg -rtsp_transport tcp -i $RTSP_URL `
    -vf "scale=${WIDTH}:${HEIGHT}:force_original_aspect_ratio=increase,crop=${WIDTH}:${HEIGHT}" `
    -c:v h264_mf `
    -b:v 500k `
    -maxrate 800k `
    -bufsize 300k `
    -g 1 `
    -pix_fmt nv12 `
    -f h264 `
    "udp://${ESP32_IP}:3333?pkt_size=1200"
