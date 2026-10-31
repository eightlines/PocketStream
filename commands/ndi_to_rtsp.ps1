# PowerShell script to stream NDI input to MediaMTX RTSP server using FFMPEG
# Edit the NDI_SOURCE and RTSP_URL as needed

$NDI_SOURCE = "TD"   # Set your NDI source name
$RTSP_URL = "rtsp://127.0.0.1:8554/live"  # Set your MediaMTX RTSP URL

# Path to ffmpeg (edit if not in PATH)
$FFMPEG = "ffmpeg"

# FFMPEG command to receive NDI and stream to RTSP
$cmd = @"
$FFMPEG -f libndi_newtek -i "$NDI_SOURCE" -vcodec libx264 -preset veryfast -tune zerolatency -pix_fmt yuv420p -f rtsp -rtsp_transport tcp "$RTSP_URL"
"@

Write-Host "Running: $cmd"
Invoke-Expression $cmd
