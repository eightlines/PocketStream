# PocketStream

An RTSP Streamer for an ESP32-P4 (Waveshare ESP32-P4-86-Panel-ETH-2RO) with TFT LCD Screen. 
Receives H.264 RTSP stream, decodes with TinyH264, displays scaled RGB565.

## Requirements

- [MediaMTX](https://mediamtx.org/docs/kickoff/install)
- [FFMPEG 8.0](https://www.gyan.dev/ffmpeg/builds/)
- [ESP32-P4-86-Panel-ETH_2RO](https://www.waveshare.com/wiki/ESP32-P4-86-Panel-ETH-2RO?srsltid=AfmBOoqReADqAAzaexSXP_GqkmBIrQq-UnQoV5d3tfqgrQohuafg43F3)
- [ESP-IDF 5.5.2](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/versions.html)
- [OBS](https://obsproject.com/)
- [obs-rtspserver Plugin](https://obsproject.com/forum/resources/obs-rtspserver.1037/)

## Usage (Test Image)

This project will configure the ESP32-P4 TFT Screen to connect to the local Ethernet network and play an RTSP stream at the endpoint `rtsp://10.0.0.1:8554/live`. This endpoint is configured in `main.c`. 

In this particular example `ffmpeg` is used to create a testpattern in the file [stream_rtsp_testpattern_esp32_720.ps1](commands/stream_rtsp_testpattern_esp32_720.ps1) and publish the output to a local RTSP service. The local RTSP service is run with [MediaMTX](https://mediamtx.org/docs/kickoff/install). 

- Build / Flash / Monitor project to the ESP32-P4 device 
- Run process `./mediamtx`
- Run FFMPEG `stream_rtsp_testpattern_esp32_720.ps1` as a separate process to publish to the RTSP service

## Usage (OBS Stream)

Using OBS, output an RTSP Stream to localhost port 8554 `/live`. This is achieved using the OBS RTSP plugin, listed above in the Requirements section. The OBS output requires the following settings to be applied before beginning the stream. Using OBS, you may input any source. In the test case I used Spout to deliver a video stream to OBS, which relays it to the ESP32. 

## OBS Settings

Settings → Output → Streaming

Video Encoder: `x264`
Rescale Output: `256x256` (Kept at a low resolution to boost the framerate)
Rate Control: `CBR`
Bitrate: `2000 Kbps`
Keyframe Interval: `1 s`
CPU Usage Preset: `slow`
Profile: `baseline`
Tune: `zerolatency`
x264 Options: `0 ref=1 keyint=30 min-keyint=30 scenecut=0 repeat-headers=1 filler=0 threads=1`

After changing x264 options: Stop then Start the RTSP plugin for settings to take effect

## Project Structure
```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── Project files
│   └── main.c
├── commands
│   ├── stream_rtsp_testpattern_esp32_720.ps1
│   ├── stream_rtsp_testpattern_esp32_720.ps1
└── README.md
```

## Notes

 If you want to scale up beyond 256×256, just change the OBS Rescale Output and update level 3.0 covers up to 720p