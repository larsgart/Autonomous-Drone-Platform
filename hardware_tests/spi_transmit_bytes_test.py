import spidev
from time import sleep

SPI_BUS = 0          # spidev0
SPI_SS  = 0          # spidev0.0
SPI_CLOCK = 4000000  # 1 Mhz
START_BYTES = [0xBC, 0x9E]

# setup SPI
spi = spidev.SpiDev(SPI_BUS, SPI_SS)
spi.max_speed_hz = SPI_CLOCK
spi.mode = 0

def crc16_ccitt(data: list):
    """
    Compute CRC16-CCITT-FALSE over a list of bytes.
    
    Args:
        data (list[int]): List of bytes (0–255) or bytes object.
    
    Returns:
        int: CRC16 result (0–65535).
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def toHex(byte_list):
   return [hex(x) for x in byte_list]

def sendData(cmd, bytes):
   crc16 = crc16_ccitt(START_BYTES + [cmd] + bytes)
   spi.xfer(START_BYTES + [cmd] + bytes + [crc16 >> 8 & 0xFF, crc16 & 0xFF])

def sendMotorSpeeds(speeds: list):
   byte_stream = [speed >> i & 0xFF for speed in speeds for i in (8, 0)]
   sendData(0x01, byte_stream)

def sendMotorStop():
   sendData(0x02, [0xFF])

def readRegister(reg, num_bytes):
   sendData(0x03, [reg])
   reg_data = spi.xfer((num_bytes+4)*[0xFF])
   reg_data = reg_data[4:]
   return reg_data

try:
   # print("Reading fw version register")
   # print(toHex(readRegister(0x00, 2)))
   # print("Arming motors")
   # sendData(0x00, [0x01])
   # sleep(0.5)
   # print("Reading arm status register")
   # print(toHex(readRegister(0x04, 1)))
   # sleep(2)
   # print("Sending speeds")
   # sendMotorSpeeds([1600, 1000, 2000, 1200])
   # sleep(0.5)
   # print("Reading motor speeds register")
   # print(toHex(readRegister(0x05, 8)))
   # sleep(2)
   # print("Sending bad speeds")
   # sendMotorSpeeds([1600, 3000, 2000, 1200])
   # sleep(0.5)
   # print("Reading motor speeds register")
   # print(toHex(readRegister(0x05, 8)))
   # sleep(2)
   # print("Stopping motors")
   # sendMotorStop()
   # sleep(0.5)
   # print("Reading motor speeds register")
   # print(toHex(readRegister(0x05, 8)))
   # sleep(2)
   # print("Disarming motors")
   # sendData(0x00, [0x00])
   # sleep(0.5)
   # print("Reading arm status register")
   # print(toHex(readRegister(0x04, 1)))
   # sleep(0.5)
   # print("Reading bad register")
   # print(toHex(readRegister(0x07, 2)))
   sendData(0x00, [0x01])
finally:
   # close SPI
   print("Closing SPI")
   spi.close()