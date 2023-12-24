import time
import board
import busio
import digitalio
import adafruit_rfm9x
import wifi
import socketpool
from adafruit_minimqtt.adafruit_minimqtt import MQTT

WIFI_SSID = 'Apollo'
WIFI_PASSWORD = 'ad2008-ad2022'

led = digitalio.DigitalInOut(board.GP5)
led.direction = digitalio.Direction.OUTPUT

def flash_led(times, duration):
    for _ in range(times):
        led.value = True
        time.sleep(duration)
        led.value = False
        time.sleep(duration)

def connect_to_wifi(ssid, password):
    while True:
        try:
            print("Trying to connect to WiFi...")
            wifi.radio.connect(ssid, password)
            print("Connected to Wi-Fi!")
            break
        except Exception as e:
            print("Failed to connect to WiFi. Retrying...")
            flash_led(2, 2)
            time.sleep(5)  # Wait for 5 seconds before retrying

connect_to_wifi(WIFI_SSID, WIFI_PASSWORD)
flash_led(4, 1)

pool = socketpool.SocketPool(wifi.radio)

MQTT_SERVER = "driver.cloudmqtt.com"
MQTT_PORT = 18756
USERNAME = "tvuuvbox"
PASSWORD = "Nqe3McF21AjF"
DEVICE_ID = "LoRaReceiver"
MQTT_TOPIC = "WeatherInfo"

# Configure SPI for RFM95
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)
cs = digitalio.DigitalInOut(board.GP17)
rst = digitalio.DigitalInOut(board.GP14)

# Initialize RFM9x
rfm9x = adafruit_rfm9x.RFM9x(spi, cs, rst, 915.0)

# Setup MQTT client
mqtt_client = MQTT(broker=MQTT_SERVER, port=MQTT_PORT, username=USERNAME, password=PASSWORD, socket_pool=pool)

# Connect to MQTT Broker
mqtt_client.connect()

print("Listening for LoRa messages...")

while True:
    packet = rfm9x.receive()
    if packet is not None:
        message = packet.decode("utf-8")
        print("Received LoRa message:", message)
        flash_led(2, 0.5)  # Flash LED 2 times, each for 0.5 seconds

        # Publish message to MQTT
        mqtt_client.publish(MQTT_TOPIC, message)
        print("Sent MQTT message:", message)

