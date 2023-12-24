from machine import Pin, PWM
import utime

class Servo:
    def __init__(self, pin):
        self.servo = PWM(Pin(pin))
        self.servo.freq(50)  # Set frequency to 50Hz, suitable for servos

    def set_position(self, value):
        int_value = int(value)
        angle = 180 - (int_value / 40) * 180
        angle = max(0, min(angle, 180))  # Ensure angle is within 0-180 range

        # Convert the angle to duty cycle
        duty = int((angle / 18) + 2)
        self.servo.duty_u16(duty * 65536 // 100)  # Convert to 16-bit value
