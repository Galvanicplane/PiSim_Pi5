# PiSim_Pi5

Raspberry Pi 5 Python scripts for UDP telemetry communication with **PiSim (Unreal Engine 5)**.

## File Structure

- `airplane.py`: `AirplaneUDPMessage` class definition with binary serialization (`to_binary`, `from_binary`) matching `FAirplaneUDPMessage` in Unreal Engine C++.
- `land_vehicle.py`: `LandVehicleUDPMessage` class definition with binary serialization (`to_binary`, `from_binary`) matching `FLandVehicleUDPMessage` in Unreal Engine C++.
- `udp_sender.py`: `UDPSender` class to send binary telemetry packets over UDP from Raspberry Pi 5 to PiSim.
- `udp_receiver.py`: `UDPReceiver` class to listen for incoming binary UDP packets, inspect packet header `MessageType`, and deserialize into corresponding vehicle struct.

## Requirements

- Python 3.8+ (Raspberry Pi OS / Linux / Windows)
- Standard library modules (`socket`, `struct`, `time`)

## Usage Example

### Sending Telemetry (Pi 5 -> Unreal Engine)
```python
from udp_sender import UDPSender
from airplane import AirplaneUDPMessage

sender = UDPSender(target_ip="192.168.1.100", target_port=5005)
plane = AirplaneUDPMessage(
    location_x=100.0, location_y=200.0, location_z=500.0,
    altitude=500.0, airspeed=120.0, payload="Telemetry OK"
)
sender.send_airplane(plane)
sender.close()
```

### Receiving Telemetry (Unreal Engine -> Pi 5)
```python
from udp_receiver import UDPReceiver

receiver = UDPReceiver(listen_ip="0.0.0.0", listen_port=5005)
msg_obj, raw_bytes, sender_addr = receiver.receive_packet(timeout=5.0)

if msg_obj:
    print(f"Received valid message from {sender_addr}: {msg_obj}")
```
