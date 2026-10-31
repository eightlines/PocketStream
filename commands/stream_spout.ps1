# Stream SpoutCam to MediaMTX RTSP server
$RTSP_URL = "rtsp://127.0.0.1:8554/live"  # Set your MediaMTX RTSP URL

ffmpeg `
    -f dshow `
    -video_size 720x720 `
    -framerate 15 `
    -i video="SpoutCam" `
    -c:v libx264 `
    -profile:v baseline `
    -level 3.0 `
    -preset veryfast `
    -tune zerolatency `
    -b:v 2M `
    -g 15 `
    -bf 0 `
    -pix_fmt yuv420p `
    -x264-params keyint=15:min-keyint=15:scenecut=0:repeat-headers=1:cabac=0 `
    -f rtsp -rtsp_transport tcp `
    $RTSP_URL