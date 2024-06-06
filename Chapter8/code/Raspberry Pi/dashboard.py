from kivy.config import Config

# Set the application to run in fullscreen mode
Config.set('graphics', 'fullscreen', 'auto')
Config.set('graphics', 'borderless', '1')

from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.clock import Clock
from threading import Thread
import paho.mqtt.client as mqtt
from kivy_garden.mapview import MapView, MapMarkerPopup
from buzzer import BuzzerMelody

MQTT_SERVER = "codfish.rmq.cloudamqp.com"
MQTT_PORT = 1883
USERNAME = "xxxxxxx" 
MQTT_PASSWORD = "xxxxxxxxxx"
DEVICE_ID = "IoTAlarmDashboard"
TOPIC = "IoTAlarm"


class AlarmDashboard(FloatLayout):

    def __init__(self, **kwargs):
        super(AlarmDashboard, self).__init__(**kwargs)
        self.password_toggle = "1234"
        self.entered_password = ''
        self.system_armed = False
        self.lockout = False
        self.last_message = ''

        self.client = mqtt.Client(client_id=DEVICE_ID)
        self.client.username_pw_set(USERNAME, MQTT_PASSWORD)
        self.client.on_message = self.on_message

        Thread(target=self.start_mqtt).start()

        self.init_widgets()
        self.buzzer = BuzzerMelody(4)

    def on_message(self, client, userdata, msg):
        message = f"{str(msg.payload.decode('utf-8'))}"
        self.last_message = "Last message: " + message
        
        if message.startswith("location:"):
            parts = message.split(":")[1].split(",")
            lat = float(parts[0])
            lon = float(parts[1])
            
            print(lat)
            print(lon)
            
            Clock.schedule_once(lambda dt: self.update_map(lat, lon), 0)

        if message == "arm":
            Clock.schedule_once(lambda dt: self.update_system_status(True), 0)
        elif message == "disarm":
            Clock.schedule_once(lambda dt: self.update_system_status(False), 0)
            
        if message == "buzzer":
            self.buzzer.play_melody()

        Clock.schedule_once(lambda dt: self.update_message_display(), 0)
        
    def update_map(self, lat, lon):
        # Set the center of the map to the new coordinates
        self.mapview.center_on(lat, lon)
        
        # Add the marker to the new location
        marker = MapMarkerPopup(lat=lat, lon=lon)
        self.mapview.add_widget(marker)


    def update_system_status(self, is_armed):
        if is_armed:
            self.system_armed = True
            self.system_status.text = "System is ARMED"
            self.system_status.color = (0, 1, 0, 1)
        else:
            self.system_armed = False
            self.system_status.text = "System is DISARMED"
            self.system_status.color = (1, 0, 0, 1)

    def start_mqtt(self):
        self.client.connect(MQTT_SERVER, MQTT_PORT)
        self.client.subscribe(TOPIC)
        self.client.loop_forever()

    def init_widgets(self):
        # Keypad buttons
        positions = [(0.03, 0.75), (0.14, 0.75), (0.25, 0.75),
                     (0.03, 0.55), (0.14, 0.55), (0.25, 0.55),
                     (0.03, 0.35), (0.14, 0.35), (0.25, 0.35)]

        for index, pos in enumerate(positions, 1):
            btn = Button(text=str(index), size_hint=(0.1, 0.1), pos_hint={'x': pos[0], 'y': pos[1]})
            btn.bind(on_press=self.handle_key_press)
            self.add_widget(btn)

        # System status
        self.system_status = Label(text="System is DISARMED", size_hint=(1, 0.2), pos_hint={'x': -0.3, 'y': 0.1}, font_size=30,
                                   color=(1, 0, 0, 1))
        self.add_widget(self.system_status)

        # MQTT Messages
        self.message_display = Label(text="Waiting for message...", size_hint=(0.77, 0.6), pos_hint={'x': 0.23, 'y': 0.62}, font_size=25,
                                     color=(1, 1, 1, 1))
        self.add_widget(self.message_display)
        
        self.mapview = MapView(zoom=15, lat=52.379189, lon=4.899431, size_hint=(0.5, 0.7), pos_hint={'x': 0.45, 'y': 0.15})
        self.add_widget(self.mapview)

        # Adding a marker
        #marker = MapMarkerPopup(lat=52.379189, lon=4.899431)
        #mapview.add_widget(marker)


    def update_message_display(self):
        self.message_display.text = self.last_message

    def handle_key_press(self, instance):
        if not self.lockout:
            self.entered_password += instance.text
            
            print("The key:" + instance.text + " was pressed")	#print keypress
            
            if len(self.entered_password) == 4:
                if self.entered_password == self.password_toggle:
                    if self.system_armed:
                        self.system_armed = False
                        self.system_status.text = "System is DISARMED"
                        self.system_status.color = (1, 0, 0, 1)
                        # Send the 'disarm' message to the IoTAlarm topic
                        self.client.publish(TOPIC, "disarm")
                    else:
                        self.system_armed = True
                        self.system_status.text = "System is ARMED"
                        self.system_status.color = (0, 1, 0, 1)
                        # Send the 'arm' message to the IoTAlarm topic
                        self.client.publish(TOPIC, "arm")
                    self.entered_password = ''
                else:
                    self.lockout = True
                    Clock.schedule_once(self.end_lockout, 5)
                    self.entered_password = ''

    def end_lockout(self, dt):
        self.lockout = False


class MyApp(App):

    def build(self):
        return AlarmDashboard()


if __name__ == '__main__':
    MyApp().run()

