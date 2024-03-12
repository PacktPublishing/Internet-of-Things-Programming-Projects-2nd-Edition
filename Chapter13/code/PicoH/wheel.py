class Wheel:
    def __init__(self, robotics_board, speed):
        self.motor_board = robotics_board
        self.speed = speed
    
    def forward(self):
        # Assuming motors 1 and 2 are for forward movement
        self.motor_board.motorOn(1, "f", self.speed)
        self.motor_board.motorOn(2, "f", self.speed)
    
    def reverse(self):
        # Assuming motors 1 and 2 are for backward movement
        self.motor_board.motorOn(1, "r", self.speed)
        self.motor_board.motorOn(2, "r", self.speed)
    
    def turn_right(self):
        # Assuming motor 1 forward and motor 2 backward for right turn
        self.motor_board.motorOn(1, "r", self.speed)
        self.motor_board.motorOn(2, "f", self.speed)
    
    def turn_left(self):
        # Assuming motor 1 backward and motor 2 forward for left turn
        self.motor_board.motorOn(1, "f", self.speed)
        self.motor_board.motorOn(2, "r", self.speed)
    
    def stop(self):
        # Stops both motors
        self.motor_board.motorOff(1)
        self.motor_board.motorOff(2)

#Test code
#wheel = Wheel()
#wheel.forward()
#time.sleep(1)
#wheel.reverse()
#time.sleep(1)
#wheel.turn_right()
#time.sleep(1)
#wheel.turn_left()
#time.sleep(1)
#wheel.stop()


 