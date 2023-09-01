from gpiozero import DistanceSensor
from time import sleep

sensor = DistanceSensor(echo=24, trigger=23)

while True:
    print('Distance: ', sensor.distance * 100, 'cm')
    sleep(1)
