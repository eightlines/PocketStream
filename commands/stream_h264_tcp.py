#!/usr/bin/env python3
"""
H.264 TCP Streamer
Reads H.264 NAL units from stdin (piped from ffmpeg) and sends to ESP32 with length-prefix framing.

Protocol: [4-byte length (big-endian)][H.264 NAL unit data]

Usage:
    ffmpeg ... -f h264 pipe:1 | python stream_h264_tcp.py <esp32_ip> <port>
"""

import sys
import socket
import struct
import time

def find_nal_units(data):
    """Find NAL unit boundaries (0x00 0x00 0x01 or 0x00 0x00 0x00 0x01)"""
    units = []
    i = 0
    start = -1
    
    while i < len(data) - 4:
        # Check for start code
        if data[i:i+3] == b'\x00\x00\x01':
            if start >= 0:
                units.append(data[start:i])
            start = i + 3
            i += 3
        elif data[i:i+4] == b'\x00\x00\x00\x01':
            if start >= 0:
                units.append(data[start:i])
            start = i + 4
            i += 4
        else:
            i += 1
    
    # Return units and remaining buffer (incomplete NAL unit)
    if start >= 0:
        return units, data[start:]
    return units, data

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <esp32_ip> <port>", file=sys.stderr)
        sys.exit(1)
    
    host = sys.argv[1]
    port = int(sys.argv[2])
    
    print(f"Connecting to {host}:{port}...", file=sys.stderr)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.connect((host, port))
    print(f"Connected!", file=sys.stderr)
    
    stdin = sys.stdin.buffer
    
    buffer = b''
    nal_count = 0
    start_time = time.time()
    last_print = start_time
    bytes_sent = 0
    
    try:
        while True:
            chunk = stdin.read(65536)
            if not chunk:
                break
            
            buffer += chunk
            
            # Extract NAL units
            units, buffer = find_nal_units(buffer)
            
            for nal in units:
                if len(nal) == 0:
                    continue
                
                # Send with Annex B start code for decoder
                nal_with_start = b'\x00\x00\x00\x01' + nal
                length = len(nal_with_start)
                
                header = struct.pack('>I', length)
                sock.sendall(header + nal_with_start)
                nal_count += 1
                bytes_sent += length
                
                now = time.time()
                if now - last_print >= 1.0:
                    elapsed = now - start_time
                    rate = bytes_sent / 1024 / elapsed if elapsed > 0 else 0
                    print(f"Sent {nal_count} NAL units, {rate:.1f} KB/s", file=sys.stderr)
                    last_print = now
    
    except (BrokenPipeError, ConnectionResetError):
        print("Connection closed", file=sys.stderr)
    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
    finally:
        sock.close()

if __name__ == '__main__':
    main()
