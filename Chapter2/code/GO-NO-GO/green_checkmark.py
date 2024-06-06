from sense_emu import SenseHat

class GreenCheck:
    
    black = (0, 0, 0)
    green = (0, 255, 0)
    
    check_mark_pixels = [
        black, black, black, black, black, black, black, green,
        black, black, black, black, black, black, green, green,
        black, black, black, black, black, black, green, green,
        black, black, black, black, black, green, green, black,
        green, black, black, black, green, green, black, black,
        black, green, black, black, green, green, black, black,
        black, green, green, green, green, black, black, black,
        black, black, black, green, black, black, black, black
    ]

    def __init__(self, rotation=0):
        
        self.sense = SenseHat()
        self.sense.set_rotation(rotation)

    
    def display(self):
        self.sense.set_pixels(self.check_mark_pixels)
   

if __name__ == "__main__":
 greenCheck = GreenCheck(rotation = 270)
 greenCheck.display()