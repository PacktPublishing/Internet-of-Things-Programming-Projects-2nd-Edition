import board
import digitalio
import analogio
import time

class Joystick:
    def __init__(self):
        self.adc_x = analogio.AnalogIn(board.GP27)
        self.adc_y = analogio.AnalogIn(board.GP26)
        
        self.button = digitalio.DigitalInOut(board.GP0)
        self.button.direction = digitalio.Direction.INPUT
        self.button.pull = digitalio.Pull.UP

        self.button2 = digitalio.DigitalInOut(board.GP22)
        self.button2.direction = digitalio.Direction.INPUT
        self.button2.pull = digitalio.Pull.UP

        self.mid = 32767
        self.dead_zone = 10000

    def get_binary_value(self, value):
        if abs(value - self.mid) < self.dead_zone:
            return 0
        return -1 if value < self.mid else 1

    def read(self):
        x_val = self.get_binary_value(self.adc_x.value)
        y_val = self.get_binary_value(self.adc_y.value)
        
        button_state = not self.button.value
        button2_state = not self.button2.value
        
        return x_val, y_val, button_state, button2_state

# Test code
if __name__ == "__main__":
    joystick = Joystick()

    while True:
        x, y, button_state, button2_state = joystick.read()
        print("X Position:", x)
        print("Y Position:", y)
        print("Button 1 Pressed:", button_state)
        print("Button 2 Pressed:", button2_state)
        time.sleep(5)
