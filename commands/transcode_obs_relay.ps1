# OBS → ESP32 transcode relay
#
# Problem: OBS encodes IDR keyframes as 4 separate slice NALs (multi-slice access unit).
# TinyH264 (ESP32 software decoder) only decodes the first IDR slice, leaving the
# reference frame 75% corrupt. P-frames then scramble from that reference.
#
# Solution: FFmpeg reads the OBS stream, re-encodes with threads=1 (single-thread x264
# = guaranteed single IDR slice), and re-publishes to /live for the ESP32.
#
# Setup (no changes to ESP32 code or OBS needed):
#   1. Make sure OBS RTSP Server plugin is running (publishing to /obs)
#   2. Run this script
#   3. ESP32 continues reading from /live as normal

$OBS_RTSP_URL   = "rtsp://127.0.0.1:8554/obs"   # obs-rtspserver publishes here
$ESP32_RTSP_URL = "rtsp://127.0.0.1:8554/live"  # ESP32 reads from here

ffmpeg -rtsp_transport tcp -i $OBS_RTSP_URL `
    -c:v libx264 -profile:v baseline -level 3.0 -pix_fmt yuv420p `
    -x264-params "keyint=30:min-keyint=30:scenecut=0:repeat-headers=1:cabac=0:threads=1:bframes=0:filler=0" `
    -bf 0 -preset ultrafast -tune zerolatency `
    -f rtsp -rtsp_transport tcp $ESP32_RTSP_URL
