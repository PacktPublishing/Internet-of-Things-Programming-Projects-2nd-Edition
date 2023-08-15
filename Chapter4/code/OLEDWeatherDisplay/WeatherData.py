import requests
import json

class WeatherData:
    
    temperature = 0
    weather_conditions = ''
    city = ''
    
    def __init__(self, city):
        self.city = city
        api_key = '5f7a09c03dc707a3e4347c48b3c60535'
        base_url = "http://api.openweathermap.org/data/2.5/weather"
        complete_url = f"{base_url}?q={self.city}&appid={api_key}&units=metric"
        response = requests.get(complete_url)
        data = response.json()
        
        if data["cod"] != "404":
            main = data["main"]
            weather = data["weather"]
            self.temperature = main["temp"]
            self.weather_conditions = weather[0]["main"]



    def get_temperature(self):
        return str(int(self.temperature))
    
    
    def get_conditions(self):
        return self.weather_conditions
    
if __name__=="__main__": 
    weather = WeatherData('Toronto')
    print(weather.get_temperature())
    print(weather.get_conditions())
