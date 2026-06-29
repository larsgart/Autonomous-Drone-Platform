import struct

# Protocol: https://github.com/house4hack/circuitpython-ibus


class IBus:
    _SERVO    = 0x40
    _CHANNELS = 14
    _OVERHEAD = 3
    _MAX_LEN  = 0x20

    def __init__(self, uart):
        self._uart = uart

    def read(self):
        length_byte = self._uart.read(1)
        expected_len = length_byte[0] - 1
        if not (self._OVERHEAD <= expected_len < self._MAX_LEN):
            return None

        payload = bytearray(expected_len)
        if self._uart.readinto(payload) != expected_len:
            return None

        if payload[0] & 0xF0 != self._SERVO:
            return None

        total = expected_len + 1 + sum(payload[:-2])
        computed = 0xFFFF - total
        stored = (payload[-1] << 8) | payload[-2]
        if stored != computed:
            return None

        return list(struct.unpack_from(f'<{self._CHANNELS}H', payload, 1))
