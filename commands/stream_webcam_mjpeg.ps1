ffmpeg -f dshow -i video="Opal Tadpole" `
    -vf "scale=720:720" -c:v h264_mf -b:v 2M -g 30 `
    -f h264 tcp://10.0.0.2:3333