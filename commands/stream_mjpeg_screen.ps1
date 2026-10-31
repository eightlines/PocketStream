# MJPEG streaming - Screen capture
# Captures desktop and streams to ESP32

$ESP32_IP = "10.0.0.2"  # Change to your ESP32's IP
$PORT = 3333

Write-Host "Streaming screen capture to ESP32 at ${ESP32_IP}:${PORT}" -ForegroundColor Cyan
Write-Host "Press Ctrl+C to stop" -ForegroundColor Yellow

# Capture screen, crop to square, stream as MJPEG
# Use yuvj444p pixel format for esp_jpeg compatibility
ffmpeg -f gdigrab -framerate 30 -i desktop `
    -vf "scale=720:720:force_original_aspect_ratio=increase,crop=720:720" `
    -pix_fmt yuvj444p -c:v mjpeg -q:v 15 `
    -f mjpeg pipe:1 2>$null | python stream_mjpeg_tcp.py $ESP32_IP $PORT
