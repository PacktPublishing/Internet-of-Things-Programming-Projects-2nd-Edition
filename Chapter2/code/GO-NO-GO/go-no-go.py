import requests
import json
import time
from green_checkmark import GreenCheck
from flashing_x import RedXAnimation

latitude = '42.346268'
longitude = '-71.095764'

go = GreenCheck(rotation=270)
no_go = RedXAnimation(rotation=270)
timer = 1
age = 12

base_url = "https://api.openweathermap.org/data/2.5/weather"
api_key = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
params = {
    'lat': latitude,
    'lon': longitude,
    'appid': api_key,
    'units': 'metric'
}

while True:
    response = requests.get(base_url, params=params)

    if response.status_code == 200:
        data = response.json()

        temperature = data['main']['temp']
        description = data['weather'][0]['main']

        print(f"The current temperature is {temperature}°C.")
        print(f"The weather is {description}.")

        if description == 'Thunderstorm' or description == 'Rain' and age < 16:
            print("NO-GO!")
            no_go.display_animation(duration=1)
            timer = 1
        else:
            print("GO!")
            go.display()
            timer = 60

    else:
        print("Error: Failed to retrieve weather information.")

    time.sleep(timer)
