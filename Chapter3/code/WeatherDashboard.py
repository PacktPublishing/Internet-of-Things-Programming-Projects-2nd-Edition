from gpiozero import Servo
from gpiozero import LED
from time import sleep
from WeatherData import WeatherData

class WeatherDashboard:
    servoCorrection = 0.5
    maxPW = (2.0 + servoCorrection) / 1000
    minPW = (1.0 - servoCorrection) / 1000

    def __init__(self, city, servo_pin, led_pin):   
        self.city = city
        self.servo = Servo(servo_pin, min_pulse_width=self.minPW, max_pulse_width=self.maxPW)
        self.led = LED(led_pin)

    def update_status(self):
        weather_data = WeatherData(self.city)

        self.servo.value = weather_data.getServoValue()
        led_status = weather_data.getLEDValue()

        if led_status == 0:
            self.led.off()
        elif led_status == 1:
            self.led.on()
        else:
            self.led.blink()

    def closeServo(self):
        self.servo.close()
    
    
def update_dashboard(city, servo_pin, led_pin):
    weather_dashboard = WeatherDashboard(city, servo_pin, led_pin)
    weather_dashboard.update_status()
    sleep(2)
    weather_dashboard.closeServo()


if __name__ == "__main__":
    city = 'Toronto'
    servo_pin = 14
    led_pin = 25

    while True:
        update_dashboard(city, servo_pin, led_pin)
        sleep(1800)  # sleep for 30 minutes