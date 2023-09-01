import Adafruit_DHT
from time import sleep
DHT_SENSOR = Adafruit_DHT.DHT11
DHT_PIN = 23

while True:
    humidity, temperature = Adafruit_DHT.read(
                                      DHT_SENSOR,
                                      DHT_PIN)
    if humidity is not None and temperature is not None:
        print(f"Temp={temperature:.2f}C                
                Humidity={humidity:.2f}%")
    else:
        print("Sensor failure. Check wiring.")
    sleep(5)
