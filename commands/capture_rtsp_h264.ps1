# Capture a short segment of the RTSP stream to a raw H.264 file for offline analysis
# Usage: .\capture_rtsp_h264.ps1

# Set RTSP URL and output file
$rtspUrl = "rtsp://127.0.0.1:8554/live"
$outputFile = "capture.h264"

# Capture 5 seconds of video
ffmpeg -rtsp_transport tcp -i $rtspUrl -t 5 -c copy -an -bsf:v h264_mp4toannexb $outputFile

Write-Host "Captured 5 seconds of RTSP stream to $outputFile."
Write-Host "You can now analyze this file with ffplay, ffprobe, or other tools."
