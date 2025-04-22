import struct
import serial
from typing import List, Optional
import time

class RH56Hand:
    def __init__(self, port: str, hand_id: int = 1, baudrate: int = 115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            stopbits=1,
            parity='N',
            timeout=1
        )
        self.hand_id = hand_id

    def _calc_checksum(self, data: bytes) -> int:
        return sum(data) & 0xFF

    def _send_frame(self, command: int, address: int, data: bytes = b'') -> Optional[bytes]:
        frame = bytes([0xEB, 0x90, self.hand_id])
        data_length = len(data) + 3
        frame += bytes([data_length, command])
        frame += struct.pack('<H', address)
        frame += data
        checksum = self._calc_checksum(frame[2:])
        frame += bytes([checksum])
        self.ser.write(frame)
        response = self.ser.read(64)
        return response if response else None

    def _parse_response(self, response: bytes) -> Optional[List[int]]:
        if len(response) < 5:
            return None
        if response[0] != 0x90 or response[1] != 0xEB:
            return None
        if response[2] != self.hand_id:
            return None
        if self._calc_checksum(response[2:-1]) != response[-1]:
            return None
        return response


    def angle_set(self, angles: List[int]):
        if len(angles) != 6:
            raise ValueError("Need 6")
        
        data = b''
        for angle in angles:
            if not (-1 <= angle <= 1000):
                raise ValueError("range error")
            data += struct.pack('<h', angle)
        
        # ANGLE_SET: 0x05CE (1486)
        response = self._send_frame(
            command=0x12,
            address=0x05CE,
            data=data
        )
        return self._parse_response(response)

    def angle_read(self) -> Optional[List[int]]:
        # ANGLE_ACT：0x060A（1546），12byte
        response = self._send_frame(
            command=0x11,
            address=0x060A,
            data=bytes([0x0C])
        )
        if parsed := self._parse_response(response):
            raw_data = parsed[7:-1]
            return [struct.unpack('<h', raw_data[i:i+2])[0] for i in range(0, 12, 2)]
        return None

    def force_act(self) -> Optional[List[int]]:
        """
        Read force sensor values from each finger
        
        Returns:
            Optional[List[int]]: List of force values for 6 fingers, or None if reading fails
            Force values typically range from 0-1000, higher values indicate greater force
        """
        # FORCE_ACT: 0x0616 (1558), 12 bytes
        response = self._send_frame(
            command=0x11,
            address=0x0616,
            data=bytes([0x0C])
        )
        if parsed := self._parse_response(response):
            raw_data = parsed[7:-1]
            return [struct.unpack('<h', raw_data[i:i+2])[0] for i in range(0, 12, 2)]
        return None

    def set_id(self, new_id: int):
        if not (1 <= new_id <= 254):
            raise ValueError("ID range error")
        # HAND_ID：0x05E8（1000）
        response = self._send_frame(
            command=0x12,
            address=0x05E8,
            data=bytes([new_id])
        )
        if self._parse_response(response):
            self.hand_id = new_id

    def clear_errors(self):
        # CLEAR_ERROR：0x05EC（1004）
        self._send_frame(
            command=0x12,
            address=0x05EC,
            data=bytes([1])
        )

    def save_parameters(self):
        # SAVE：0x05ED（1005）
        self._send_frame(
            command=0x12,
            address=0x05ED,
            data=bytes([1])
        )

    def smooth_angle_set(self, target_angles: List[int], steps: int = 30, delay: float = 0.05):
        """
        Smoothly move fingers to target angles by dividing the movement into small steps
        
        Args:
            target_angles: List of target angles (6 values)
            steps: Number of steps to divide the movement into (default 30)
            delay: Delay time between steps in seconds (default 0.05)
        """
        if len(target_angles) != 6:
            raise ValueError("Need 6 angle values")
        
        # Read current angles
        current_angles = self.angle_read()
        if current_angles is None:
            raise RuntimeError("Failed to read current angles")
        
        # Calculate angle increment for each step
        angle_steps = []
        for curr, target in zip(current_angles, target_angles):
            diff = target - curr
            step = diff / steps
            angle_steps.append(step)
        
        # Execute step by step
        for i in range(steps):
            next_angles = []
            for j, (curr, step) in enumerate(zip(current_angles, angle_steps)):
                if i == steps - 1:  # Use target angle for the last step
                    next_angles.append(target_angles[j])
                else:
                    next_angle = int(curr + step * (i + 1))
                    next_angles.append(next_angle)
            
            self.angle_set(next_angles)
            time.sleep(delay)

if __name__ == "__main__":
    hand = RH56Hand(port="/dev/ttyUSB0", hand_id=1)
    
    #hand.angle_set([0, 0, 0, 0, 0, 0])
    
    print("angle:", hand.angle_read())
    forces = hand.force_act()
    if forces:
        print("Force values for each finger:", forces)
    #hand.set_id(2)
    #hand.save_parameters()
