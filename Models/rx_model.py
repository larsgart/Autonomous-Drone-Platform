import serial
import numpy as np
from ibus_model import IBus

DEADBAND_LOW  = 1492
DEADBAND_HIGH = 1508
DEADBAND_CENTER = 1500
MISSED_READING_LIMIT = 100


class RX:
    def __init__(self):
        self._uart = serial.Serial(port="/dev/ttyTHS1", baudrate=115200)
        self._ibus = IBus(self._uart)
        self._output = np.array([1500, 1500, 1000, 1500])
        self._missed = 0

    def read(self) -> np.ndarray:
        self._uart.reset_input_buffer()
        packet = self._ibus.read()

        if packet:
            self._output = np.clip(packet, 1000, 2000)
            self._missed = 0
        else:
            self._missed += 1
            if self._missed > MISSED_READING_LIMIT:
                self._output = np.array([1500, 1500, 1000, 1500])

        for ch in (0, 1, 3):
            v = self._output[ch]
            self._output[ch] = DEADBAND_CENTER if DEADBAND_LOW < v < DEADBAND_HIGH else v

        return self._output

    def read_normalized(self) -> np.ndarray:
        return (self.read() - 1000) / 1000

    def close(self):
        if self._uart and self._uart.is_open:
            self._uart.close()

    def __del__(self):
        self.close()
