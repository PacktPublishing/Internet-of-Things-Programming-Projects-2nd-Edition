import time
import board
import busio
import digitalio
import adafruit_rfm9x
import adafruit_dht

# Configure SPI for RFM95
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
cs = digitalio.DigitalInOut(board.GP17)
rst = digitalio.DigitalInOut(board.GP14)

# Initialize RFM9x
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, rst, 915.0)

# Initialize DHT22
dht_sensor = adafruit_dht.DHT22(board.GP4)

print("Sending temperature and humidity data every 60 seconds")

while True:
    try:
        temperature = dht_sensor.temperature
        humidity = dht_sensor.humidity
        data = f"Temp: {temperature}C, Humidity: {humidity}%"
        rfm9x.send(bytes(data, "utf-8"))
        print("Sent:", data)
    except RuntimeError as e:
        print("DHT22 read error:", e)
    
    time.sleep(120)  # Wait for 120 seconds
