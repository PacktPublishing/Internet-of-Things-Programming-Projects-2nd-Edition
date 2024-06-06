from sense_hat import SenseHat
import time

sense_hat = SenseHat()
speed = 0.05
sense_hat.set_rotation(270)
red = [255, 0, 0]
green = [0, 255, 0]

while True:
    sense.show_message("Temperature: %.1fC, " % sense.get_temperature(), 
                       scroll_speed=scroll_speed, text_colour=blue)
    sense.show_message("Humidity: %.1f%%, " % sense.get_humidity(), 
                       scroll_speed=scroll_speed, text_colour=green)
    sense.show_message("Pressure: %.1fhPa" % sense.get_pressure(), 
                       scroll_speed=scroll_speed, text_colour=blue)

    time.sleep(1)

while True:
    sense_hat.show_message("Temperature: %.1fC, " %
        sense_hat.get_temperature(),
        scroll_speed=speed,
        text_colour=green)
    
    sense_hat.show_message("Humidity: %.1f%%, " %
        sense.get_humidity(),
        scroll_speed=speed,
        text_colour=green)
    
    sense_hat.show_message("Pressure: %.1fhPa" %
        sense.get_pressure(),
        scroll_speed=speed,
    text_colour=red)

    time.sleep(1)
