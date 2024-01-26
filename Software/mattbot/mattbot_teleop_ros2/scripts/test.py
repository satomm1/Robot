import spidev
import time


# This is a minimal script for demonstrating SPI communication between the Jetson and the MCU
# Sends the message 01011001 11111111 to the MCU, and prints the response every 0.5 seconds

print("Hello World!")
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 15000
spi.mode = 0b11

try:
    while True:
        
        resp = spi.xfer2([0b01011001, 0b11111111])
        print(resp)
        time.sleep(0.5)
except KeyboardInterrupt:
    spi.close()
