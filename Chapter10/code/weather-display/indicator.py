from machine import Pin
import utime

class Indicator:
    def __init__(self):
        self.red = Pin(15, Pin.OUT)
        self.green = Pin(13, Pin.OUT)
        self.blue = Pin(12, Pin.OUT)
    
    def set_color(self, r, g, b):
        self.red.value(r)
        self.green.value(g)
        self.blue.value(b)

    def set_indicator(self, value):
        # Turn off all LEDs initially
        self.set_color(1, 1, 1)

        if value <= 30:
            # Turn on red LED
            self.set_color(0, 1, 1)
        elif 30 < value <= 50:
            # Turn on green LED
            self.set_color(1, 0, 1)
        else:
            # Turn on blue LED
            self.set_color(1, 1, 0)
            
    def flash_led(self, times):
        for _ in range(times):
            # Set LED to white
            self.set_color(0, 0, 0)

            utime.sleep(0.5)

            # Turn off the LED
            self.set_color(1, 1, 1)

            utime.sleep(0.5)
