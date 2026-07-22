# PiSim_Pi5

Raspberry Pi 5 Python scripts for telemetry communication and **live camera video streaming** with **PiSim (Unreal Engine 5)**.

## File Structure

- `airplane.py`: `AirplaneUDPMessage` struct class with binary serialization (`to_binary`, `from_binary`) matching `FAirplaneUDPMessage` in Unreal Engine C++.
- `land_vehicle.py`: `LandVehicleUDPMessage` struct class with binary serialization (`to_binary`, `from_binary`) matching `FLandVehicleUDPMessage` in Unreal Engine C++.
- `udp_sender.py`: `UDPSender` class to send binary telemetry packets over UDP (Port 5005) from Raspberry Pi 5 to PiSim.
- `udp_receiver.py`: `UDPReceiver` class to listen for incoming binary UDP telemetry packets (Port 5005).
- `video_receiver.py`: **Live OpenCV Video Receiver** to capture and display real-time camera feeds sent from Unreal Engine over Ethernet (Port 5006).

## Requirements

- Python 3.8+ (Raspberry Pi OS / Linux / Windows)
- Packages:
  ```bash
  pip install opencv-python numpy
  ```

---

## 📽️ Live Video Stream Usage (Unreal Engine -> Pi 5)

1. Run `video_receiver.py` on Raspberry Pi 5:
   ```bash
   python video_receiver.py
   ```
2. In Unreal Engine 5 (PiSim):
   - Add `UDPVideoStreamer` component to your Vehicle / Airplane Actor.
   - Attach a `SceneCaptureComponent2D` with a `TextureRenderTarget2D` (e.g. 640x480 resolution).
   - Set `TargetIP` to your Pi 5 IP (e.g., `192.168.1.100`), `TargetPort = 5006`, `FrameRate = 30`.
   - Click **Play**. The live video window will open in Python on your Pi 5!

---

## 📡 Telemetry Usage Example (Port 5005)

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
