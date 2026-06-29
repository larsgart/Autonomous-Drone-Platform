import spidev
import time

SPI_BUS = 0          # spidev0
SPI_SS  = 0          # spidev0.0
SPI_CLOCK = 1000000  # 1 Mhz

# setup SPI
spi = spidev.SpiDev(SPI_BUS, SPI_SS)
spi.max_speed_hz = SPI_CLOCK

# transfer 2 bytes at a time, ^C to exit
try:
    v = 0
    while True:
        send = [v, v+1]
        print("")
        print("TX:", send)
        print("RX:", spi.xfer(send))
        time.sleep(0.5)
        if v >= 254:
            v = 0
        else:
            v = (v+2)
finally:
    spi.close()