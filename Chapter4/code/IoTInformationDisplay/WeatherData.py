import requests
import json

class WeatherData:
    
    temperature = 0
    weather_conditions = ''
    wind_speed = 0
    city = ''
    
    def __init__(self, city):
        self.city = city
        api_key = 'xxxxxxxxxxxxxxxxxx'
        base_url = "http://api.openweathermap.org/data/2.5/weather"
        complete_url = f"{base_url}?q={self.city}&appid={api_key}&units=metric"
        response = requests.get(complete_url)
        data = response.json()
        
        if data["cod"] != "404":
            main = data["main"]
            wind = data["wind"]
            weather = data["weather"]
            self.temperature = main["temp"]
            self.weather_conditions = weather[0]["main"]
            self.wind_speed = wind["speed"]
            self.icon = weather[0]["icon"]

    def get_conditions(self):
        return self.weather_conditions
    
    def get_temperature(self):
        return str(int(self.temperature))

    def get_weather_conditions_icon(self):
        return f"http://openweathermap.org/img/wn/{self.icon}.png"

    def get_wind_temp_factor(self):
        if self.temperature < 0:
            temp_factor = 0
        elif self.temperature > 30:
            temp_factor = 30
        else:
            temp_factor = self.temperature

        wind_factor = self.wind_speed / 20
        wind_temp_factor = temp_factor - wind_factor
        
        return wind_temp_factor

    def get_attire_image(self):
        factor = self.get_wind_temp_factor()
        if factor < 8:
            return "images/gloves.png"
        elif factor < 18:
            return "images/long-shirt.png"
        elif factor < 25:
            return "images/short-shirt.png"
        else:
            return "images/shorts.png"
      
if __name__=="__main__": 
    weather = WeatherData('Toronto')
    print(weather.get_temperature())
    print(weather.get_attire_image())
    print(weather.get_conditions())
    print(weather.get_weather_conditions_icon())