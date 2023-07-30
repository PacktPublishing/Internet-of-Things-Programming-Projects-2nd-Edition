import requests
import json

class WeatherData:
    
    temperature = 0
    weather_conditions = ''
    wind_speed = 0
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
            wind = data["wind"]
            weather = data["weather"]
            self.temperature = main["temp"]
            self.weather_conditions = weather[0]["main"]
            self.wind_speed = wind["speed"]
    
    def getServoValue(self):
        if self.temperature < 0:
            temp_factor = 0
        elif self.temperature > 30:
            temp_factor = 1
        else:
            temp_factor = self.temperature  / 30

        wind_factor = self.wind_speed / 20
        servo_value = -((temp_factor - (wind_factor / 20)) * 2 - 1)
        
        return servo_value

    
    def getLEDValue(self):   
        if (self.weather_conditions=='Thunderstorm'):
            return 2
        elif(self.weather_conditions=='Rain'):
            return 1
        else:
            return 0
    
if __name__=="__main__": 
    weather = WeatherData('Toronto')
    print(weather.getServoValue())
    print(weather.getLEDValue())