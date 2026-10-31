# MJPEG streaming to ESP32-P4 (Max Performance)
# Uses hardware JPEG decoder + TCP transport + double buffering

$ESP32_IP = "10.0.0.2"  # Change to your ESP32's IP
$PORT = 3333

Write-Host "Streaming 720x720 MJPEG to ESP32 at ${ESP32_IP}:${PORT}" -ForegroundColor Cyan
Write-Host "Press Ctrl+C to stop" -ForegroundColor Yellow

# Stream webcam as MJPEG via TCP with length-prefix framing
# - 720x720 resolution (native to display)
# - yuvj444p pixel format for esp_jpeg compatibility
# - MJPEG quality 15 (~50-100KB per frame, good quality)
ffmpeg -f dshow -rtbufsize 256M -framerate 30 -video_size 1280x720 -i video="USB Video" `
    -vf "scale=720:720:force_original_aspect_ratio=increase,crop=720:720" `
    -pix_fmt yuvj444p -c:v mjpeg -q:v 15 `
    -f mjpeg pipe:1 2>$null | python stream_mjpeg_tcp.py $ESP32_IP $PORT
