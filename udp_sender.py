import socket
import time
from airplane import AirplaneUDPMessage
from land_vehicle import LandVehicleUDPMessage

class UDPSender:
    """
    UDP Sender utility for Raspberry Pi 5 to send telemetry messages to PiSim (Unreal Engine).
    """
    def __init__(self, target_ip: str = "127.0.0.1", target_port: int = 5005):
        self.target_ip = target_ip
        self.target_port = target_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_bytes(self, data: bytes) -> bool:
        """Send raw binary bytes to the target UDP IP and port."""
        try:
            bytes_sent = self.sock.sendto(data, (self.target_ip, self.target_port))
            return bytes_sent == len(data)
        except Exception as e:
            print(f"[UDPSender] Error sending data: {e}")
            return False

    def send_airplane(self, msg: AirplaneUDPMessage) -> bool:
        """Serialize and send an AirplaneUDPMessage packet."""
        return self.send_bytes(msg.to_binary())

    def send_land_vehicle(self, msg: LandVehicleUDPMessage) -> bool:
        """Serialize and send a LandVehicleUDPMessage packet."""
        return self.send_bytes(msg.to_binary())

    def close(self):
        """Close the socket connection."""
        self.sock.close()

if __name__ == "__main__":
    print("Starting UDP Sender Test...")
    sender = UDPSender(target_ip="127.0.0.1", target_port=5005)

    # Example 1: Sending Airplane Telemetry
    plane = AirplaneUDPMessage(location_x=120.5, location_y=450.0, location_z=1500.0, altitude=1500.0, airspeed=240.5, payload="Flight Check OK")
    print(f"Sending Airplane: {plane}")
    sender.send_airplane(plane)

    time.sleep(0.5)

    # Example 2: Sending Land Vehicle Telemetry
    car = LandVehicleUDPMessage(steering_angle=15.0, speed=65.0, headlights_on=True, payload="Cruising")
    print(f"Sending Land Vehicle: {car}")
    sender.send_land_vehicle(car)

    sender.close()
    print("Finished sending.")
