import network
import utime
from umqtt.simple import MQTTClient
from servo import Servo
from indicator import Indicator

class WeatherDisplay:
    def __init__(self):
        # WiFi Information
        self.ssid = "Apollo"
        self.wifi_password = "ad2008-ad2022"

        # MQTT Information
        self.mqtt_server = "driver.cloudmqtt.com"
        self.mqtt_port = 18756
        self.username = "tvuuvbox"
        self.mqtt_password = "Nqe3McF21AjF"
        self.device_id = "WeatherDisplay"
        self.mqtt_topic = "WeatherInfo"

        self.indicator = Indicator()
        self.servo = Servo(14)
        
    def connect_wifi(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        if not wlan.isconnected():
            print('Connecting to WiFi...')
            wlan.connect(self.ssid, self.wifi_password)
            while not wlan.isconnected():
                pass
        print('WiFi connected, IP:', wlan.ifconfig()[0])
        # Flash the indicator four times upon successful connection
        self.indicator.flash_led(4)

        

    def connect_mqtt(self):
        self.client = MQTTClient(self.device_id, self.mqtt_server, self.mqtt_port, self.username, self.mqtt_password)
        self.client.set_callback(self.on_message_received)
        self.client.connect()
        self.client.subscribe(self.mqtt_topic)

    def on_message_received(self, topic, msg):
        print("Received:", topic, msg.decode())
        temperature, humidity = self.parse_message(msg)
        if temperature is not None:
            self.servo.set_position(temperature)
        if humidity is not None:
            self.indicator.set_indicator(humidity)

    def parse_message(self, msg):
        try:
            parts = msg.decode().split(',')
            temperature_str = parts[0].split('Temp:')[1].split('C')[0].strip()
            humidity_str = parts[1].split('Humidity:')[1].split('%')[0].strip()
            
            temperature = float(temperature_str)
            humidity = float(humidity_str)
            return temperature, humidity
        except Exception as e:
            print("Error parsing message:", str(e))
            return None, None

    def run(self):
        self.connect_wifi()
        self.connect_mqtt()
        while True:
            try:
                self.client.check_msg()
            except Exception as e:
                print("Error checking MQTT message:", str(e))
                utime.sleep(5)

# Create and run the weather display
weather_display = WeatherDisplay()
weather_display.run()
