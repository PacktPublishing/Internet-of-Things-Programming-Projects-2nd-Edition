from machine import Pin, PWM
import utime

class Servo:
    def __init__(self, pin):
        self.servo = PWM(Pin(pin))
        self.servo.freq(50)  # Set frequency to 50Hz, suitable for servos

    def set_position(self, angle):
        # Reverse the angle
        reversed_angle = 180 - angle

        # Convert the reversed angle to duty cycle
        duty = int((reversed_angle / 18) + 2)
        self.servo.duty_u16(duty * 65536 // 100)  # Convert to 16-bit value
