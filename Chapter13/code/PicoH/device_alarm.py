import time
import board
import pwmio
import digitalio

class Alarm:
    def __init__(self, buzzer_pin=board.GP1, led_pin1=board.GP0, led_pin2=board.GP2, frequency=4000):
        # Initialize buzzer
        self.buzzer = pwmio.PWMOut(buzzer_pin, frequency=frequency, duty_cycle=0)
        
        # Initialize first LED
        self.led1 = digitalio.DigitalInOut(led_pin1)
        self.led1.direction = digitalio.Direction.OUTPUT
        
        # Initialize second LED
        self.led2 = digitalio.DigitalInOut(led_pin2)
        self.led2.direction = digitalio.Direction.OUTPUT
    
    def activate_alarm(self, num_of_times=5):
        blink_rate = 0.5  # Half a second on, half a second off
        
        for _ in range(num_of_times):
            # Buzzer on and both LEDs on
            #self.buzzer.duty_cycle = 32768  # 50% duty cycle
            self.led1.value = True
            self.led2.value = True
            time.sleep(blink_rate)
            
            # Buzzer off and both LEDs off
            #self.buzzer.duty_cycle = 0
            self.led1.value = False
            self.led2.value = False
            time.sleep(blink_rate)

# Example usage:
#alarm = Alarm(buzzer_pin=board.GP1, led_pin1=board.GP0, led_pin2=board.GP2)
#alarm.activate_alarm(10)
