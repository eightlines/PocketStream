# TouchDesigner → ESP32 transcode relay
#
# Use this ONLY if direct TD → ESP32 still fails after the M-bit fix.
# TD's 256x256 @ 15fps stream should work directly with the M-bit workaround.
#
# If you still need a relay (e.g. to cap bitrate or rescale):
#   1. Download MediaMTX from https://github.com/bluenviron/mediamtx/releases
#      and run it with default config (serves RTSP on port 8555 if you edit mediamtx.yml
#      to change rtspAddress: :8555)
#   2. Run this script — FFmpeg reads TD, re-encodes, publishes to MediaMTX
#   3. Flash ESP32 with RTSP_PORT = 8555 (already set in main.c)
#
# Note: -rtsp_flags server was removed in FFmpeg 8.x — MediaMTX is required as relay target.

$TD_URL    = "rtsp://127.0.0.1:8554/live"   # TD's embedded LIVE555 server
$RELAY_URL = "rtsp://127.0.0.1:8555/live"   # MediaMTX relay target (port 8555)

ffmpeg `
    -rtsp_transport tcp -i $TD_URL `
    -c:v libx264 -profile:v baseline -level 3.0 -pix_fmt yuv420p `
    -b:v 500k -maxrate 500k -bufsize 1000k `
    -x264-params "keyint=30:min-keyint=30:scenecut=0:repeat-headers=1:cabac=0:threads=1:bframes=0:filler=0" `
    -bf 0 -preset ultrafast -tune zerolatency `
    -f rtsp -rtsp_transport tcp $RELAY_URL
