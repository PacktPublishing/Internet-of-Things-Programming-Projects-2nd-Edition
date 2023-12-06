import time
import board
import busio
import digitalio
import adafruit_rfm9x

# Configure SPI for RFM95
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
cs = digitalio.DigitalInOut(board.GP17)
rst = digitalio.DigitalInOut(board.GP14)

# Initialize RFM9x
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, rst, 915.0)

# Initialize LED
led = digitalio.DigitalInOut(board.GP5)
led.direction = digitalio.Direction.OUTPUT

print("Listening for LoRa messages...")

def flash_led(times, duration):
    for _ in range(times):
        led.value = True
        time.sleep(duration)
        led.value = False
        time.sleep(duration)

while True:
    packet = rfm9x.receive()
    if packet is not None:
        print("Received message:", packet)
        flash_led(2, 0.5)  # Flash LED 2 times, each for 0.5 seconds
