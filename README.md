# PocketStream

An RTSP Streamer for an ESP32-P4 with TFT LCD Screen. 

## Requirements

- [MediaMTX](https://mediamtx.org/docs/kickoff/install)
- [FFMPEG 8.0](https://www.gyan.dev/ffmpeg/builds/)
- [ESP32-P4-86-Panel-ETH_2RO](https://www.waveshare.com/wiki/ESP32-P4-86-Panel-ETH-2RO?srsltid=AfmBOoqReADqAAzaexSXP_GqkmBIrQq-UnQoV5d3tfqgrQohuafg43F3)
- [ESP-IDF 5.5.2](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/versions.html)

## Usage

This project will configure the ESP32-P4 TFT Screen to connect to the local Ethernet network and play an RTSP stream at the endpoint `rtsp://10.0.0.1:8554/live`. This endpoint is configured in `main.c`. 

In this particular example `ffmpeg` is used to create a testpattern in the file [stream_rtsp_testpattern_esp32_720.ps1](commands/stream_rtsp_testpattern_esp32_720.ps1) and publish the output to a local RTSP service. The local RTSP service is run with [MediaMTX](https://mediamtx.org/docs/kickoff/install). 

- Build / Flash / Monitor project to the ESP32-P4 device 
- Run process `./mediamtx`
- Run FFMPEG `stream_rtsp_testpattern_esp32_720.ps1` as a separate process to publish to the RTSP service

### Result

The ESP32-P4 device will display the RTSP stream on the device. 

## Project Structure
```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── Project files
│   └── main.c
├── commands
│   ├── stream_rtsp_testpattern_esp32_720.ps1
└── README.md
```
