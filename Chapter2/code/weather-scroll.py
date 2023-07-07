import requests
import json
from sense_emu import SenseHat
import time

api_key = 'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx'
location = 'Toronto'

base_url = 'https://api.openweathermap.org/data/2.5/weather'
params = {
    'q': location,
    'appid': api_key,
    'units': 'metric'
}

sense = SenseHat()
sense.set_rotation(270)

last_call_time = time.time() - 30  # Set initial time to allow immediate first call
last_weather_info = ""

while True:
    current_time = time.time()

    if current_time - last_call_time >= 30:
        response = requests.get(base_url, params=params)
        data = response.json()

        temperature = data['main']['temp']
        description = data['weather'][0]['description']
        weather_info = f"{location}: {temperature}°C, {description}"

        last_weather_info = weather_info

        sense.show_message(weather_info, scroll_speed=0.05, text_colour=[255, 255, 255])
        last_call_time = current_time

    else:
        sense.show_message(last_weather_info, scroll_speed=0.05, text_colour=[255, 255, 255])

    time.sleep(1)
