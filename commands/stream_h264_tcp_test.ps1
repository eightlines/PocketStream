# H.264 streaming over TCP to ESP32-P4
# Reliable transport - raw H.264 stream

$ESP32_IP = "10.0.0.2"  # Change to your ESP32's IP
$PORT = 3333

Write-Host "Streaming 320x320 H.264 to ESP32 at ${ESP32_IP}:${PORT}" -ForegroundColor Cyan
Write-Host "Press Ctrl+C to stop" -ForegroundColor Yellow

# Generate test pattern, encode as H.264, stream directly via TCP
ffmpeg -f lavfi -i "testsrc=size=320x320:rate=30" `
    -pix_fmt yuv420p `
    -c:v libx264 -preset ultrafast -tune zerolatency `
    -profile:v baseline -level 3.0 `
    -x264-params "keyint=1:scenecut=0" `
    -b:v 1M `
    -f h264 tcp://${ESP32_IP}:${PORT}
