from PicoRobotics import KitronikPicoRobotics
import time

class Wheel:
    def __init__(self, speed):
        self.motor_board = KitronikPicoRobotics()
        self.speed = speed
    
    def forward(self):
        self.motor_board.motorOn(1, "f", self.speed)
        self.motor_board.motorOn(2, "f", self.speed)
    
    def reverse(self):
        self.motor_board.motorOn(1, "r", self.speed)
        self.motor_board.motorOn(2, "r", self.speed)
    
    def turn_right(self):
        self.motor_board.motorOn(1, "r", self.speed)
        self.motor_board.motorOn(2, "f", self.speed)
    
    def turn_left(self):
        self.motor_board.motorOn(1, "f", self.speed)
        self.motor_board.motorOn(2, "r", self.speed)
    
    def stop(self):
        # Stops both motors
        self.motor_board.motorOff(1)
        self.motor_board.motorOff(2)

#Test code
#wheel = Wheel(20)
#wheel.forward()
#time.sleep(10)
#wheel.reverse()
#time.sleep(1)
#wheel.turn_right()
#time.sleep(1)
#wheel.turn_left()
#time.sleep(1)
#wheel.stop()


 