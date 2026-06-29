import serial
import random
import numpy as np


class Motors:
    def __init__(self):
        self.test_passed = False
        self._uart = None
        self._connect()

    def _connect(self):
        print("Waiting for MCU ENQ...")
        self._uart = serial.Serial(port="/dev/ttyS0", baudrate=115200)
        while self._uart.read() != b'\x05':
            pass
        print("Connected to MCU")
        self._uart.write(bytearray([6]))  # ACK

    def test_motors(self):
        print("Testing motors")
        for _ in range(100):
            speeds = [random.randint(0, 100) for _ in range(4)]
            self.output_speeds(speeds)
            received = [int((int(x) - 1000) / 10)
                        for x in self._uart.read_until().decode().strip().split(",")]
            if received != speeds:
                print(f"Test failed: expected {speeds}, got {received}")
                self._uart.write(bytearray([21]))  # NACK
                return False

        self._uart.write(bytearray([6]))   # ACK
        self._uart.write(bytearray([4]))   # EOT
        print("Motor tests passed")
        self.zero_throttle()
        self.test_passed = True
        return True

    def output_speeds(self, speeds: list):
        if not self._validate(speeds):
            return
        scaled = [int(np.clip(s * 10 + 1000, 1000, 2000)) for s in speeds]
        stream = [2] + [b for s in scaled for b in (s >> 8, s & 0xFF)]
        self._uart.write(bytearray(stream))

    def zero_throttle(self):
        self.output_speeds([0] * 4)

    def close(self):
        if self._uart and self._uart.is_open:
            self._uart.write(bytearray([27]))  # ESC → MCU back to idle
            self._uart.close()

    def __del__(self):
        self.close()

    def _validate(self, speeds) -> bool:
        return len(speeds) == 4 and all(0 <= s <= 100 for s in speeds)
