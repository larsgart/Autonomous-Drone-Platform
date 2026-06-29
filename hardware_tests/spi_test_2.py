from periphery import SPI
import ctypes

SPI_DEVICE = "/dev/spidev0.0"
SPI_MODE = 0
SPI_SPEED = 100000

spi = SPI(SPI_DEVICE, SPI_MODE, SPI_SPEED)

libc = ctypes.CDLL('libc.so.6')

try:
  data_in_1 = spi.transfer([0xaa, 0xbb])
  data_in_2 = spi.transfer([0xee, 0xff])

  print(data_in_1)
  print(data_in_2)

finally:
  spi.close()