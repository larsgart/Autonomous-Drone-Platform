import struct
import logging
import serial
import numpy as np

log = logging.getLogger(__name__)

# iBUS protocol — https://github.com/house4hack/circuitpython-ibus
_SERVO    = 0x40
_CHANNELS = 14
_OVERHEAD = 3
_MAX_LEN  = 0x20

DEADBAND_LOW    = 1492
DEADBAND_HIGH   = 1508
DEADBAND_CENTER = 1500
MISSED_LIMIT    = 100


def _read_ibus(uart) -> list | None:
    expected_len = uart.read(1)[0] - 1
    if not (_OVERHEAD <= expected_len < _MAX_LEN):
        return None

    payload = bytearray(expected_len)
    if uart.readinto(payload) != expected_len:
        return None

    if payload[0] & 0xF0 != _SERVO:
        return None

    computed = 0xFFFF - (expected_len + 1 + sum(payload[:-2]))
    if (payload[-1] << 8) | payload[-2] != computed:
        return None

    return list(struct.unpack_from(f'<{_CHANNELS}H', payload, 1))


class RX:
    def __init__(self):
        self._uart = serial.Serial(port="/dev/ttyTHS1", baudrate=115200)
        self._output = np.array([1500, 1500, 1000, 1500])
        self._missed = 0

    def read(self) -> np.ndarray:
        self._uart.reset_input_buffer()
        packet = _read_ibus(self._uart)

        if packet:
            self._output = np.clip(packet, 1000, 2000)
            self._missed = 0
        else:
            self._missed += 1
            if self._missed > MISSED_LIMIT:
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
