import struct

class AirplaneUDPMessage:
    """
    Airplane-specific UDP Message for communication with PiSim (Unreal Engine).
    Matches FAirplaneUDPMessage struct layout in C++.
    """
    def __init__(self, message_type: str = "AirplaneTelemetry", 
                 location_x: float = 0.0, location_y: float = 0.0, location_z: float = 0.0,
                 altitude: float = 0.0, airspeed: float = 0.0, payload: str = ""):
        self.message_type = message_type
        self.location_x = float(location_x)
        self.location_y = float(location_y)
        self.location_z = float(location_z)
        self.altitude = float(altitude)
        self.airspeed = float(airspeed)
        self.payload = payload

    def set_location(self, x: float, y: float, z: float):
        """Helper to set airplane 3D location vector."""
        self.location_x = float(x)
        self.location_y = float(y)
        self.location_z = float(z)

    def to_binary(self) -> bytes:
        """Serialize struct data into binary byte format compatible with Unreal Engine C++."""
        msg_bytes = self.message_type.encode('utf-8')
        payload_bytes = self.payload.encode('utf-8')

        data = bytearray()
        # MessageType string length + bytes
        data.extend(struct.pack('<I', len(msg_bytes)))
        data.extend(msg_bytes)

        # Location X, Y, Z, Altitude, Airspeed as 32-bit floats (Little Endian)
        data.extend(struct.pack('<5f', self.location_x, self.location_y, self.location_z, self.altitude, self.airspeed))

        # Payload string length + bytes
        data.extend(struct.pack('<I', len(payload_bytes)))
        data.extend(payload_bytes)

        return bytes(data)

    @classmethod
    def from_binary(cls, data: bytes) -> 'AirplaneUDPMessage':
        """Deserialize binary byte buffer into an AirplaneUDPMessage object."""
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

        # Read 5 floats: LocX, LocY, LocZ, Altitude, Airspeed
        if len(data) < offset + 20: # 5 * 4 bytes
            raise ValueError("Buffer too short for flight telemetry floats")
        loc_x, loc_y, loc_z, alt, speed = struct.unpack_from('<5f', data, offset)
        offset += 20

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
                   location_x=loc_x, location_y=loc_y, location_z=loc_z,
                   altitude=alt, airspeed=speed, payload=payload)

    def __repr__(self):
        return (f"<AirplaneUDPMessage type='{self.message_type}' "
                f"loc=({self.location_x:.2f}, {self.location_y:.2f}, {self.location_z:.2f}) "
                f"alt={self.altitude:.2f} speed={self.airspeed:.2f} payload='{self.payload}'>")
