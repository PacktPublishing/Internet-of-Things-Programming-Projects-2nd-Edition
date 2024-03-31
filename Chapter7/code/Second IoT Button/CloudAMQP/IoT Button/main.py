from machine import Pin, I2C
import utime
import network
from umqtt.simple import MQTTClient
from buzzer import play_notes
import ssd1306
import _thread

SSID = "Apollo"
WIFI_PASSWORD = "ad2008-ad2022"

led = machine.Pin(16, machine.Pin.OUT)
button = Pin(0, Pin.IN, Pin.PULL_UP)
switch = Pin(1, Pin.IN, Pin.PULL_UP)
previous_switch_state = switch.value()


MQTT_SERVER = "codfish.rmq.cloudamqp.com"
MQTT_PORT = 1883
USERNAME = "zbqtrhqa:zbqtrhqa"
PASSWORD = "1k1y20xoK00lLm5Wt5qq3YG_DQ99abLO"
DEVICE_ID = "IoTAlarmSystem"
last_message = ""

i2c = I2C(0, scl=Pin(9), sda=Pin(8))
display = ssd1306.SSD1306_I2C(128, 64, i2c)

mqtt_client = None

def on_message_received(topic, msg):
    global last_message
    print("Received:", topic, msg)
    if topic == b"IoTAlarm":
        last_message = msg.decode()
        if msg == b"buzzer":
            play_notes()
            
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    # Check if not already connected
    if not wlan.isconnected():
        print('Connecting to WiFi...')
        wlan.connect(SSID, WIFI_PASSWORD)
        
        # Blink the LED while trying to connect
        while not wlan.isconnected():
            led.on()
            utime.sleep(0.5)
            led.off()
            utime.sleep(0.5)
            
        # Turn on LED steadily when connected
        led.on()
        print('WiFi connected, IP:', wlan.ifconfig()[0])

def connect_mqtt():
    global mqtt_client
    while mqtt_client is None:
        try:
            print('Trying to connect to MQTT Server...')
            mqtt_client = MQTTClient(DEVICE_ID, MQTT_SERVER, MQTT_PORT, USERNAME, PASSWORD)
            mqtt_client.set_callback(on_message_received)
            mqtt_client.connect()
            mqtt_client.subscribe(b"IoTAlarm")
            print('MQTT connection established and subscribed to IoTAlarm')
        except:
            mqtt_client = None
            print('Failed to connect to MQTT Server, retrying...')
            utime.sleep(5)

def display_status():
    global last_message
    is_armed = False

    while True:
        display.fill(0)
        
        if mqtt_client:
            msg = "MQTT Connected"
        else:
            msg = "MQTT waiting"
        display.text(msg, 0, 0)
        
        if last_message == "arm":
            is_armed = True
        elif last_message == "disarm":
            is_armed = False
        if is_armed:
            display.text("Status: Armed", 0, 20)
        else:
            display.text("Status: Disarmed", 0, 20)
        
        display.text("Msg: " + last_message, 0, 40)
        
        display.show()
        utime.sleep(5)


def main():
    global last_message, previous_switch_state
    connect_wifi()
    connect_mqtt()
    button_start_time = None

    while True:
        # Button check
        if button.value() == 0:
            if button_start_time is None:
                button_start_time = utime.ticks_ms()
        else:
            if button_start_time is not None:
                button_elapsed_time = utime.ticks_diff(utime.ticks_ms(), button_start_time)
                if button_elapsed_time >= 1000:
                    mqtt_client.publish(b"IoTAlarm", b"arm")
                    last_message = "arm"
                button_start_time = None

        # Switch check
        current_switch_state = switch.value()
        if current_switch_state != previous_switch_state:
            mqtt_client.publish(b"IoTAlarm", b"disarm")
            last_message = "disarm"
        previous_switch_state = current_switch_state

        try:
            mqtt_client.check_msg()
        except Exception as e:
            print("Error checking MQTT message:", str(e))
            utime.sleep(5)
        
        utime.sleep(0.1)

_thread.start_new_thread(display_status, ())
main()
