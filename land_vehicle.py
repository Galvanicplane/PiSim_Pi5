import struct

class LandVehicleUDPMessage:
    """
    Land Vehicle-specific UDP Message for communication with PiSim (Unreal Engine).
    Matches FLandVehicleUDPMessage struct layout in C++.
    """
    def __init__(self, message_type: str = "LandVehicleTelemetry",
                 steering_angle: float = 0.0, speed: float = 0.0,
                 headlights_on: bool = False, payload: str = ""):
        self.message_type = message_type
        self.steering_angle = float(steering_angle)
        self.speed = float(speed)
        self.headlights_on = bool(headlights_on)
        self.payload = payload

    def toggle_headlights(self):
        """Helper to toggle vehicle headlights status."""
        self.headlights_on = not self.headlights_on

    def to_binary(self) -> bytes:
        """Serialize struct data into binary byte format compatible with Unreal Engine C++."""
        msg_bytes = self.message_type.encode('utf-8')
        payload_bytes = self.payload.encode('utf-8')

        data = bytearray()
        # MessageType string length + bytes
        data.extend(struct.pack('<I', len(msg_bytes)))
        data.extend(msg_bytes)

        # SteeringAngle (float), Speed (float), HeadlightsOn (uint8)
        headlights_val = 1 if self.headlights_on else 0
        data.extend(struct.pack('<ffB', self.steering_angle, self.speed, headlights_val))

        # Payload string length + bytes
        data.extend(struct.pack('<I', len(payload_bytes)))
        data.extend(payload_bytes)

        return bytes(data)

    @classmethod
    def from_binary(cls, data: bytes) -> 'LandVehicleUDPMessage':
        """Deserialize binary byte buffer into a LandVehicleUDPMessage object."""
        offset = 0

        # Read MessageType string
        if len(data) < offset + 4:
            raise ValueError("Buffer too short for message type length")
        msg_type_len = struct.unpack_from('<I', data, offset)[0]
        offset += 4

        if len(data) < offset + msg_type_len:
            raise ValueError("Buffer too short for message type payload")
        message_type = data[offset:offset + msg_type_len].decode('utf-8')
        offset += msg_type_len

        # Read SteeringAngle (float), Speed (float), HeadlightsOn (uint8)
        if len(data) < offset + 9: # 4 + 4 + 1 bytes
            raise ValueError("Buffer too short for land vehicle telemetry data")
        steering, speed, headlights_val = struct.unpack_from('<ffB', data, offset)
        offset += 9

        # Read Payload string
        if len(data) < offset + 4:
            raise ValueError("Buffer too short for payload length")
        payload_len = struct.unpack_from('<I', data, offset)[0]
        offset += 4

        payload = ""
        if payload_len > 0:
            if len(data) < offset + payload_len:
                raise ValueError("Buffer too short for payload string")
            payload = data[offset:offset + payload_len].decode('utf-8')

        return cls(message_type=message_type,
                   steering_angle=steering, speed=speed,
                   headlights_on=(headlights_val != 0), payload=payload)

    def __repr__(self):
        return (f"<LandVehicleUDPMessage type='{self.message_type}' "
                f"steering={self.steering_angle:.2f} speed={self.speed:.2f} "
                f"headlights={self.headlights_on} payload='{self.payload}'>")
