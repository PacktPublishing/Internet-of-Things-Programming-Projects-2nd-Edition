import time
import board
import wifi
import socketpool
import digitalio
from adafruit_minimqtt.adafruit_minimqtt import MQTT
from joystick import Joystick

WIFI_SSID = 'MySSID'
WIFI_PASSWORD = 'ssid-password'
MQTT_SERVER = "driver.cloudmqtt.com"
MQTT_PORT = 18756
USERNAME = "mqtt-username"
PASSWORD = "mqtt-password"
MQTT_TOPIC = "JoystickPosition"

led = digitalio.DigitalInOut(board.GP5)
led.direction = digitalio.Direction.OUTPUT

def flash_led(times, duration):
    for _ in range(times):
        led.value = True
        time.sleep(duration)
        led.value = False
        time.sleep(duration)

def connect_to_wifi():
    while True:
        try:
            print("Trying to connect to WiFi...")
            wifi.radio.connect(WIFI_SSID, WIFI_PASSWORD)
            print("Connected to Wi-Fi!")
            break
        except Exception as e:
            print("Failed to connect to WiFi. Retrying...")
            flash_led(1, 2)
            time.sleep(3)

def connect_to_mqtt(mqtt_client):
    while True:
        try:
            print("Trying to connect to MQTT server...")
            mqtt_client.connect()
            print("Connected to MQTT server!")
            break
        except Exception as e:
            print("Failed to connect to MQTT. Retrying...")
            flash_led(1, 0.5)
            time.sleep(3)

connect_to_wifi()

pool = socketpool.SocketPool(wifi.radio)

mqtt_client = MQTT(broker=MQTT_SERVER, port=MQTT_PORT, 
                   username=USERNAME, password=PASSWORD, 
                   socket_pool=pool)

connect_to_mqtt(mqtt_client)

led.value = True

joystick = Joystick()

def send_mqtt_message(x, y, button1, button2):
    button1_state = 1 if button1 else 0
    button2_state = 1 if button2 else 0
    message = f'X: {x}, Y: {y}, Button 1: \
              {button1_state}, Button 2: {button2_state}'
    mqtt_client.publish(MQTT_TOPIC, message)

def main():
    while True:
        x, y, button1_pressed, button2_pressed = joystick.read()
        send_mqtt_message(x, y, button1_pressed, button2_pressed)
        time.sleep(1)

if __name__ == "__main__":
    main()
