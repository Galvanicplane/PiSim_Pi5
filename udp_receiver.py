import socket
import struct
from airplane import AirplaneUDPMessage
from land_vehicle import LandVehicleUDPMessage

class UDPReceiver:
    """
    UDP Receiver utility for Raspberry Pi 5 to receive and parse telemetry messages from PiSim.
    Reads header MessageType from binary packets and parses corresponding vehicle struct.
    """
    def __init__(self, listen_ip: str = "0.0.0.0", listen_port: int = 5005):
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.listen_ip, self.listen_port))
        print(f"[UDPReceiver] Listening on {self.listen_ip}:{self.listen_port}...")

    @staticmethod
    def inspect_message_type(data: bytes) -> str:
        """Read the MessageType string header from the first bytes of any packet."""
        if len(data) < 4:
            return ""
        msg_type_len = struct.unpack_from('<I', data, 0)[0]
        if len(data) < 4 + msg_type_len:
            return ""
        return data[4:4 + msg_type_len].decode('utf-8')

    def receive_packet(self, timeout: float = 2.0):
        """
        Receive a packet and attempt to parse it based on its header MessageType.
        Returns (parsed_object, raw_bytes, sender_address).
        """
        self.sock.settimeout(timeout)
        try:
            data, addr = self.sock.recvfrom(65507)
            msg_type = self.inspect_message_type(data)

            if msg_type == "AirplaneTelemetry":
                plane_msg = AirplaneUDPMessage.from_binary(data)
                return plane_msg, data, addr
            elif msg_type == "LandVehicleTelemetry":
                car_msg = LandVehicleUDPMessage.from_binary(data)
                return car_msg, data, addr
            else:
                print(f"[UDPReceiver] Unknown or invalid message type: '{msg_type}' from {addr}")
                return None, data, addr
        except socket.timeout:
            return None, None, None

    def close(self):
        """Close the socket."""
        self.sock.close()

if __name__ == "__main__":
    receiver = UDPReceiver(listen_ip="0.0.0.0", listen_port=5005)
    print("Listening for incoming UDP packets for 10 seconds...")
    import time
    start_time = time.time()
    while time.time() - start_time < 10:
        msg_obj, raw_bytes, addr = receiver.receive_packet(timeout=1.0)
        if msg_obj:
            print(f"Received from {addr}: {msg_obj}")
        elif raw_bytes:
            print(f"Received unparsed packet from {addr} ({len(raw_bytes)} bytes)")
    receiver.close()
    print("Receiver finished.")
