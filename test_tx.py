import socket
import sys

target = sys.argv[1] if len(sys.argv) > 1 else "255.255.255.255"
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.sendto(b"SELAM_PISIM_TEST", (target, 7400))
print(f"🚀 Paket firlatildi -> {target}:7400")
