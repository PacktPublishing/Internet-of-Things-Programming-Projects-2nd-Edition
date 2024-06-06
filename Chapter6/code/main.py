#Please note that code may be slightly different than the book due to code improvements
import machine
import utime
import network
import _thread
from umqtt.simple import MQTTClient
from buzzer import activate_buzzer

SSID = "SSID"
PASSWORD = "password"
MQTT_SERVER = "broker.mqtthq.com"
MQTT_PORT = 1883

pir = machine.Pin(26, machine.Pin.IN)
led = machine.Pin(21, machine.Pin.OUT)

wlan = network.WLAN(network.STA_IF)
mqtt_client = None

def connect_wifi():
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    
    while not wlan.isconnected():
        print('Trying to connect to WiFi...')
        utime.sleep(5)
    
    print('WIFI connection established')

def sub_iotalarm(topic, msg):
    print((topic, msg))
    if topic == b'IoTAlarm' and msg == b'buzzer':
        print("buzzer is detected")
        activate_buzzer()

def motion_handler(pin):
    print('Motion detected!!')
    if mqtt_client is not None:
        mqtt_client.publish(b"IoTAlarm", b"motion")
    else:
        print("MQTT client is not connected.")

def connect_mqtt(device_id, callback):
    global mqtt_client
    
    while mqtt_client is None:
        try:
            print('Trying to connect to MQTT Server...')
            mqtt_client = MQTTClient(device_id, MQTT_SERVER, MQTT_PORT)
            mqtt_client.set_callback(callback)
            mqtt_client.connect()
            print('MQTT connection established')
        except:
            mqtt_client = None
            print('Failed to connect to MQTT Server, retrying...')
            utime.sleep(5)

def connection_status():
    while True:
        if wlan.isconnected():
            if mqtt_client is not None:
                led.on()  # Steady on when both WiFi and MQTT connected
            else:
                led.on()  # Blink every half-second when only WiFi is connected
                utime.sleep(0.5)
                led.off()
                utime.sleep(0.5)
        else:
            led.on()  # Blink every second when WiFi is not connected
            utime.sleep(1)
            led.off()
            utime.sleep(1)

_thread.start_new_thread(connection_status, ())
connect_wifi()
connect_mqtt("IoTAlarmSystem", sub_iotalarm)
mqtt_client.subscribe("IoTAlarm")
pir.irq(trigger=machine.Pin.IRQ_RISING, handler=motion_handler)

while True:
    mqtt_client.wait_msg()
