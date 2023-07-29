from gpiozero import Servo
from gpiozero import LED
from time import sleep
from WeatherData import WeatherData

class WeatherDashboard:
    
    servoCorrection=0.5
    maxPW=(2.0+servoCorrection)/1000
    minPW=(1.0-servoCorrection)/1000

    
    def __init__(self, city, servo_pin, led_pin):   
        self.city = city
        self.servo_pin = servo_pin
        self.led_pin = led_pin


    def update_status(self):
        weather_data = WeatherData(self.city)

        servo=Servo(self.servoPin, 
                    min_pulse_width = WeatherDashboard.minPW, 
                    max_pulse_width = WeatherDashboard.maxPW)
        
        led = LED(self.led_pin)

        servo.value = weather_data.getServoValue()
        led_status = weather_data.getLEDValue()

        if(led_status==0):
            led.off()
        elif (led_status==1):
            led.on()
        else:
            led.blink()

        servo.close()
    
    
if __name__=="__main__":
    
    weather_dashboard = WeatherDashboard('Toronto', 14, 25)

    while True:
        weather_dashboard.update_status()
        #sleep for 30 minutes
        sleep(1800)    