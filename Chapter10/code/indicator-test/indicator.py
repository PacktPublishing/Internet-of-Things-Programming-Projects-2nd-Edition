from machine import Pin
import utime

red = Pin(15, Pin.OUT)
green = Pin(13, Pin.OUT)
blue = Pin(12, Pin.OUT)

def set_color(r, g, b):
    red.value(r)
    green.value(g)
    blue.value(b)

while True:
    # Red
    set_color(0, 1, 1)
    utime.sleep(1)
    
    # Green
    set_color(1, 0, 1)
    utime.sleep(1)
    
    # Blue
    set_color(1, 1, 0)
    utime.sleep(1)
