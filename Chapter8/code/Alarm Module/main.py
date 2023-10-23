import machine
import utime
import network
import _thread
from umqtt.simple import MQTTClient
from buzzer import activate_buzzer

SSID = "IoTProgProjects"
WIFI_PASSWORD = "programmingprojects"

MQTT_SERVER = "driver.cloudmqtt.com"
MQTT_PORT = 18915
USERNAME = "tzykbprg"
MQTT_PASSWORD = "gajEsYkOIR71"
DEVICE_ID = "IoTAlarmModule"

pir = machine.Pin(26, machine.Pin.IN)
led = machine.Pin(21, machine.Pin.OUT)

wlan = network.WLAN(network.STA_IF)
mqtt_client = None

# Global ARMED variable initialization
ARMED = False

def connect_wifi():
    wlan.active(True)
    wlan.connect(SSID, WIFI_PASSWORD)
    
    while not wlan.isconnected():
        print('Trying to connect to WiFi...')
        utime.sleep(5)
    
    print('WIFI connection established')

def sub_iotalarm(topic, msg):
    global ARMED

    print((topic, msg))

    if topic != b'IoTAlarm':
        return

    if msg == b'buzzer':
        activate_buzzer()
    elif msg == b'arm':
        ARMED = True
        print("Alarm armed!")
    elif msg == b'disarm':
        ARMED = False
        print("Alarm disarmed!")


def motion_handler(pin):
    print('Motion detected!')
    if mqtt_client:
        if ARMED:
            activate_buzzer()
            mqtt_client.publish(b"IoTAlarm", b"buzzer")
            mqtt_client.publish(b"IoTAlarm", b"location:43.70180,-79.83249")
        else:
            mqtt_client.publish(b"IoTAlarm", b"motion")
    else:
        print("MQTT client not connected.")

def connect_mqtt(device_id, callback):
    global mqtt_client
    
    while mqtt_client is None:
        try:
            print('Trying to connect to MQTT Server...')
            mqtt_client = MQTTClient(device_id, MQTT_SERVER, MQTT_PORT, USERNAME, MQTT_PASSWORD)
            mqtt_client.set_callback(callback)
            mqtt_client.connect()
            print('MQTT connection established')
        except:
            mqtt_client = None
            print('Failed to connect to MQTT Server, retrying...')
            utime.sleep(5)

def status():
    while True:
        if ARMED:
            led.on()  # LED on
            utime.sleep(0.5)  # Blink for half a second
            led.off()
            utime.sleep(4.5)  # Wait for 4.5 seconds to complete 5 seconds
        elif wlan.isconnected():
            if mqtt_client is not None:
                led.on()  # Steady on when both WiFi and MQTT connected
                utime.sleep(5)
            else:
                led.on()  # Blink every half-second when only WiFi is connected
                utime.sleep(0.5)
                led.off()
                utime.sleep(4.5)
        else:
            led.on()
            utime.sleep(2)
            led.off()
            utime.sleep(3)

_thread.start_new_thread(status, ())
connect_wifi()
connect_mqtt(DEVICE_ID, sub_iotalarm)
mqtt_client.subscribe(b"IoTAlarm")
pir.irq(trigger=machine.Pin.IRQ_RISING, handler=motion_handler)

while True:
    mqtt_client.wait_msg()
