from sense_hat import SenseHat
import time

sense = SenseHat()
scroll_speed = 0.05
sense.set_rotation(270)
blue = [0, 0, 255]
green = [0, 255, 0]

while True:
    sense.show_message("Temperature: %.1fC, " % sense.get_temperature(), \
                       scroll_speed=scroll_speed, text_colour=blue)
    sense.show_message("Humidity: %.1f%%, " % sense.get_humidity(), \
                       scroll_speed=scroll_speed, text_colour=green)
    sense.show_message("Pressure: %.1fhPa" % sense.get_pressure(), \
                       scroll_speed=scroll_speed, text_colour=blue)

    time.sleep(1)
