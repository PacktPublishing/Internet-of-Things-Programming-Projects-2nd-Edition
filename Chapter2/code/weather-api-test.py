import requests

url = "https://api.openweathermap.org/data/2.5/weather"
api_key = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

location = "Toronto"

params = {
    "q": location,
    "appid": api_key,
    "units": "metric"
}

response = requests.get(url, params=params)

if response.status_code == 200:
    data = response.json()

    temperature = data["main"]["temp"]
    description = data["weather"][0]["description"]

    print(f"The current temperature in {location} is {temperature}°C.")
    print(f"The weather is {description}.")
else:
    print("Error: Failed to retrieve weather information.")
