from gpiozero import MotionSensor, Button, Buzzer
from time import sleep

# Initialize components
pir = MotionSensor(23)
button = Button(7)
buzzer = Buzzer(21)

# State variable
active = False

def toggle_alarm():
    global active
    if active:
        active = False
        buzzer.off()
        print("Alarm deactivated!")
    else:
        active = True
        print("Alarm activated!")

def monitor():
    while True:
        if active:
            pir.wait_for_motion()
            print("Motion detected!")
            sleep(5)  # Delay before alarm triggers
            if active:  # Check again to see if alarm was deactivated during delay
                buzzer.on()
                print("Alarm triggered!")
        else:
            buzzer.off()

button.when_pressed = toggle_alarm

try:
    monitor()
except KeyboardInterrupt:
    print("Exiting...")
    buzzer.off()
