# Monitor raw microphone audio from ESP32 via UDP (port 9001)
# ESP32 streams mono 16-bit PCM at 16 kHz continuously.
# Requires ffplay (part of ffmpeg) in PATH.
#
# Usage:
#   .\commands\monitor_mic.ps1           <- live playback
#   .\commands\monitor_mic.ps1 -Record   <- record to mic.wav instead

param(
    [switch]$Record,
    [string]$Port = "9001",
    [string]$Output = "mic.wav"
)

$udpUrl = "udp://0.0.0.0:$Port"
# ffmpeg 8.x removed -ac as input option; use -ch_layout mono instead
$fmt    = @("-f", "s16le", "-ar", "16000", "-ch_layout", "mono")

if ($Record) {
    Write-Host "Recording to $Output  (Ctrl+C to stop)..."
    ffmpeg @fmt -i $udpUrl $Output
} else {
    Write-Host "Playing live audio from ESP32 mic  (Ctrl+C to stop)..."
    Write-Host "  Source: $udpUrl  |  16 kHz mono 16-bit PCM"
    ffplay @fmt -nodisp $udpUrl
}
