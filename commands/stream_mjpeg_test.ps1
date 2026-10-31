# MJPEG streaming - Test pattern
# Generates test pattern for testing without camera

$ESP32_IP = "10.0.0.2"  # Change to your ESP32's IP
$PORT = 3333

Write-Host "Streaming test pattern to ESP32 at ${ESP32_IP}:${PORT}" -ForegroundColor Cyan
Write-Host "Press Ctrl+C to stop" -ForegroundColor Yellow

# Generate test pattern - try smaller size first for debugging
# Use baseline JPEG (not progressive) with 4:2:2 subsampling
ffmpeg -f lavfi -i "testsrc=size=320x320:rate=30" `
    -pix_fmt yuvj422p -c:v mjpeg -q:v 5 `
    -f mjpeg pipe:1 2>$null | python stream_mjpeg_tcp.py $ESP32_IP $PORT
