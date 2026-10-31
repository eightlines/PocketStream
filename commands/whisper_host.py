#!/usr/bin/env python3
"""
whisper_host.py — Receive PCM audio from ESP32 and transcribe with Whisper.

The ESP32 sends a 16-byte header followed by raw int16 PCM samples.
This script listens on TCP port 9000, receives the audio, transcribes it
locally via faster-whisper (or openai-whisper), and prints the result.

Install dependencies:
    pip install faster-whisper
    # — or —
    pip install openai-whisper

Usage:
    python whisper_host.py [--port 9000] [--model small] [--bind 0.0.0.0]

The script keeps running and handles one recording per connection.
"""

import argparse
import socket
import struct
import io
import numpy as np
import wave
import time
import sys

# ── Header format (must match speech_pcm_header_t in speech.c) ───────────────
# 0x41554449 "AUDI" | sample_rate | channels | bits | num_samples
HDR_FMT     = "<IIHHII"   # note: packed, little-endian
HDR_SIZE    = struct.calcsize(HDR_FMT)  # = 16 bytes
HDR_MAGIC   = 0x41554449

# ─────────────────────────────────────────────────────────────────────────────

def recv_exact(conn, n):
    """Read exactly n bytes from a connected socket."""
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise EOFError("Connection closed before receiving all data")
        buf += chunk
    return buf


def receive_audio(conn):
    """Receive header + PCM from one ESP32 connection.

    Returns (samples_np, sample_rate) or raises on error.
    """
    raw_hdr = recv_exact(conn, HDR_SIZE)
    magic, sample_rate, channels, bits, num_samples = struct.unpack(HDR_FMT, raw_hdr)

    if magic != HDR_MAGIC:
        raise ValueError(f"Bad magic: 0x{magic:08X} (expected 0x{HDR_MAGIC:08X})")

    print(f"  Header: {sample_rate} Hz, {channels}ch, {bits}-bit, {num_samples} samples "
          f"({num_samples / sample_rate:.1f} s)")

    pcm_bytes  = recv_exact(conn, num_samples * (bits // 8))
    dtype      = np.int16 if bits == 16 else np.int32
    samples    = np.frombuffer(pcm_bytes, dtype=dtype).astype(np.float32) / 32768.0
    return samples, sample_rate


def transcribe_faster_whisper(samples_f32, sample_rate, model_name):
    """Transcribe using faster-whisper (CTranslate2 backend)."""
    from faster_whisper import WhisperModel
    model = WhisperModel(model_name, compute_type="int8")
    segments, info = model.transcribe(samples_f32, beam_size=5,
                                      language="en", task="transcribe")
    text = " ".join(seg.text.strip() for seg in segments)
    return text


def transcribe_openai_whisper(samples_f32, sample_rate, model_name):
    """Transcribe using the original openai-whisper (PyTorch backend)."""
    import whisper, tempfile, os
    model = whisper.load_model(model_name)

    # openai-whisper expects a WAV file path
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        tmp_path = f.name
    try:
        pcm16 = (samples_f32 * 32767).astype(np.int16)
        with wave.open(tmp_path, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
            wf.writeframes(pcm16.tobytes())
        result = model.transcribe(tmp_path, language="en")
        return result["text"].strip()
    finally:
        os.unlink(tmp_path)


def main():
    parser = argparse.ArgumentParser(description="ESP32 Whisper transcription host")
    parser.add_argument("--port",  type=int, default=9000,
                        help="TCP port to listen on (default 9000)")
    parser.add_argument("--bind",  default="0.0.0.0",
                        help="Bind address (default 0.0.0.0)")
    parser.add_argument("--model", default="small",
                        help="Whisper model: tiny/base/small/medium/large (default: small)")
    parser.add_argument("--backend", choices=["faster", "openai"], default="faster",
                        help="Whisper backend (default: faster)")
    args = parser.parse_args()

    # Pre-load the model once so the first transcription is fast
    print(f"Loading Whisper model '{args.model}' ({args.backend}-whisper)...")
    try:
        if args.backend == "faster":
            from faster_whisper import WhisperModel
            _model = WhisperModel(args.model, compute_type="int8")
            print("  faster-whisper ready.")
        else:
            import whisper
            _model = whisper.load_model(args.model)
            print("  openai-whisper ready.")
    except ImportError as e:
        print(f"ERROR: {e}")
        print("Install with: pip install faster-whisper   — or —   pip install openai-whisper")
        sys.exit(1)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port))
    srv.listen(1)
    print(f"\nListening on {args.bind}:{args.port}  (say \"Hi ESP\" on the device)\n")

    while True:
        conn, addr = srv.accept()
        print(f"[{time.strftime('%H:%M:%S')}] Connection from {addr[0]}:{addr[1]}")
        try:
            with conn:
                samples, sr = receive_audio(conn)

            t0 = time.time()
            if args.backend == "faster":
                text = transcribe_faster_whisper(samples, sr, args.model)
            else:
                text = transcribe_openai_whisper(samples, sr, args.model)

            elapsed = time.time() - t0
            print(f"\n  Transcription ({elapsed:.1f} s):\n  >> {text}\n")

        except Exception as exc:
            print(f"  Error: {exc}")


if __name__ == "__main__":
    main()
