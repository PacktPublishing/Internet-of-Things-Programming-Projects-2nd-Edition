from kivy.config import Config

# Set the application to run in fullscreen mode
Config.set('graphics', 'fullscreen', 'auto')
Config.set('graphics', 'borderless', '1')

from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.image import Image, AsyncImage
from kivy.clock import Clock
from WeatherData import WeatherData
from TrafficMap import TrafficMap


class Dashboard(FloatLayout):
    def __init__(self, city, latitude, longitude, zoom):
        super(Dashboard, self).__init__()
        self.city = city
        self.traffic_map = TrafficMap(latitude, longitude, zoom)

        self.init_widgets()
        Clock.schedule_interval(self.update_status, 1800)
        self.update_status(0)

    def init_widgets(self):
        # Static widgets
        self.add_widget(Label(text="Temperature", pos=(-275, 175),
                              color=(1, 1, 1, 1), font_size=22,
                              font_name='fonts/ArialBlack.ttf'))
        self.add_widget(Label(text="Conditions", pos=(-275, 60),
                              color=(1, 1, 1, 1), font_size=22,
                              font_name='fonts/ArialBlack.ttf'))
        self.add_widget(Label(text="Attire", pos=(-280, -80),
                              color=(1, 1, 1, 1), font_size=22,
                              font_name='fonts/ArialBlack.ttf'))
        self.add_widget(Image(source='images/box.png', pos=(-275, 145)))
        self.add_widget(Image(source='images/box.png', pos=(-275, 10)))
        self.add_widget(Image(source='images/box.png', pos=(-275, -127)))

        # Dynamic widgets
        self.city_label = Label(text=self.city, pos=(250, 185),
                                color=(1, 1, 1, 1), font_size=30,
                                font_name='fonts/ArialBlack.ttf')
        self.add_widget(self.city_label)
        self.temperature_label = Label(pos=(-275, 125), color=(1, 1, 1, 1),
                                       font_size=40, font_name='fonts/ArialBlack.ttf')
        self.add_widget(self.temperature_label)
        self.conditions_image = AsyncImage(pos=(-278, 20))
        self.add_widget(self.conditions_image)
        self.weather_conditions_label = Label(pos=(-280, -25),
                                              color=(1, 1, 1, 1), font_size=20,
                                              font_name='fonts/ArialBlack.ttf')
        self.add_widget(self.weather_conditions_label)
        self.traffic_map_image = AsyncImage(pos=(120, -30))
        self.add_widget(self.traffic_map_image)
        self.attire_image = Image(pos=(-280, -140))
        self.add_widget(self.attire_image)

    def update_status(self, *args):
        weather_data = WeatherData(self.city)
        self.traffic_map_image.source = self.traffic_map.get_traffic_map()
        self.attire_image.source = weather_data.get_attire_image()
        self.temperature_label.text = weather_data.get_temperature() + "\u00B0C"
        self.weather_conditions_label.text = weather_data.get_conditions()
        self.conditions_image.source = weather_data.get_weather_conditions_icon()


class MyApp(App):
    def build(self):
        city = 'Toronto'
        latitude = 43.6426
        longitude = -79.3871
        zoom = 12
        return Dashboard(city, latitude, longitude, zoom)


if __name__ == '__main__':
    MyApp().run()
