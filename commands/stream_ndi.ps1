ffmpeg -f libndi_newtek -i "TouchDesigner" `
    -vf "scale=160:128" -c:v h264_mf -b:v 500k -g 5 `
    -f h264 udp://10.0.0.2:3333?pkt_size=1400