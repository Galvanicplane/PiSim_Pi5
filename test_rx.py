import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 7400))
print("=== UDP 7400 DINLENIYOR (Paket bekleniyor...) ===")

while True:
    data, addr = sock.recvfrom(1024)
    print(f"✅ GELDI! Kaynak: {addr[0]} | Boyut: {len(data)} bayt | Icerik: {data[:40]}")
