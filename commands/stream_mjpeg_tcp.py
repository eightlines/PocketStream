#!/usr/bin/env python3
"""
MJPEG TCP Streamer
Reads MJPEG frames from stdin (piped from ffmpeg) and sends to ESP32 with length-prefix framing.

Protocol: [4-byte length (big-endian)][JPEG data]

Usage:
    ffmpeg ... -f mjpeg pipe:1 | python stream_mjpeg_tcp.py <esp32_ip> <port>
"""

import sys
import socket
import struct
import time

JPEG_SOI = b'\xff\xd8'  # Start of Image marker
JPEG_EOI = b'\xff\xd9'  # End of Image marker

def find_jpeg_frames(data):
    """Extract complete JPEG frames from buffer"""
    frames = []
    while True:
        # Find start of JPEG
        soi = data.find(JPEG_SOI)
        if soi == -1:
            break
        
        # Find end of JPEG (after SOI)
        eoi = data.find(JPEG_EOI, soi + 2)
        if eoi == -1:
            # Incomplete frame, keep remainder
            data = data[soi:]
            break
        
        # Extract complete frame (including EOI marker)
        frame = data[soi:eoi + 2]
        frames.append(frame)
        data = data[eoi + 2:]
    
    return frames, data

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <esp32_ip> <port>", file=sys.stderr)
        sys.exit(1)
    
    host = sys.argv[1]
    port = int(sys.argv[2])
    
    print(f"Connecting to {host}:{port}...", file=sys.stderr)
    
    # Connect to ESP32
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.connect((host, port))
    print(f"Connected!", file=sys.stderr)
    
    # Read stdin in binary mode
    stdin = sys.stdin.buffer
    
    buffer = b''
    frame_count = 0
    start_time = time.time()
    last_print = start_time
    
    try:
        while True:
            # Read chunk from ffmpeg
            chunk = stdin.read(65536)
            if not chunk:
                break
            
            buffer += chunk
            
            # Extract complete JPEG frames
            frames, buffer = find_jpeg_frames(buffer)
            
            for frame in frames:
                # Send length-prefixed frame
                length = len(frame)
                header = struct.pack('>I', length)  # Big-endian 4-byte length
                sock.sendall(header + frame)
                frame_count += 1
                
                # Print stats every second
                now = time.time()
                if now - last_print >= 1.0:
                    elapsed = now - start_time
                    fps = frame_count / elapsed if elapsed > 0 else 0
                    print(f"Sent {frame_count} frames, {fps:.1f} fps, last frame: {length} bytes", file=sys.stderr)
                    last_print = now
    
    except (BrokenPipeError, ConnectionResetError):
        print("Connection closed by ESP32", file=sys.stderr)
    except KeyboardInterrupt:
        print("\nInterrupted", file=sys.stderr)
    finally:
        sock.close()
        elapsed = time.time() - start_time
        print(f"Total: {frame_count} frames in {elapsed:.1f}s ({frame_count/elapsed:.1f} fps)", file=sys.stderr)

if __name__ == '__main__':
    main()
