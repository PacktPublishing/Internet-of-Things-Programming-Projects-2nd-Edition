from gpiozero import MotionSensor
from time import sleep
pir = MotionSensor(23)

while True:
    if pir.motion_detected:
        print("Motion detected!!")
    else:
        print("Waiting……")
    sleep(5)
