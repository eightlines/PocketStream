import socket
import time

IP = "192.168.137.237"
PORT = 3333
MESSAGE = b"Hello ESP32 Video Stream Test"

print(f"Sending UDP packets to {IP}:{PORT}...")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    for i in range(10):
        sock.sendto(MESSAGE, (IP, PORT))
        print(f"Sent packet {i+1}")
        time.sleep(0.5)
except Exception as e:
    print(f"Error: {e}")
finally:
    sock.close()
    print("Done.")
