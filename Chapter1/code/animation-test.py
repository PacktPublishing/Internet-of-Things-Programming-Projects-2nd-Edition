from sense_hat import SenseHat
import time

sense = SenseHat()

R = [255, 0, 0]  # Red
O = [255, 165, 0]  # Orange
Y = [255, 255, 0]  # Yellow
B = [0, 0, 0]  # Black

frame1 = [
B, B, B, B, B, B, B, B,
B, B, B, B, B, B, B, B,
B, B, B, R, R, B, B, B,
B, B, R, R, R, R, B, B,
B, B, R, R, R, R, B, B,
B, B, B, R, R, B, B, B,
B, B, B, B, B, B, B, B,
B, B, B, B, B, B, B, B
]

frame2 = [
B, B, Y, O, O, Y, B, B,
B, Y, O, R, R, O, Y, B,
Y, O, R, Y, Y, R, O, Y,
O, R, Y, B, B, Y, R, O,
O, R, Y, B, B, Y, R, O,
Y, O, R, Y, Y, R, O, Y,
B, Y, O, R, R, O, Y, B,
B, B, Y, O, O, Y, B, B
]

frame3 = [
B, Y, O, R, R, O, Y, B,
Y, O, R, Y, Y, R, O, Y,
O, R, Y, O, O, Y, R, O,
R, Y, O, B, B, O, Y, R,
R, Y, O, B, B, O, Y, R,
O, R, Y, O, O, Y, R, O,
Y, O, R, Y, Y, R, O, Y,
B, Y, O, R, R, O, Y, B
]

frame4 = [
O, R, R, Y, Y, R, R, O,
R, Y, O, O, O, O, Y, R,
Y, O, B, B, B, B, O, Y,
O, B, B, B, B, B, B, O,
O, B, B, B, B, B, B, O,
Y, O, B, B, B, B, O, Y,
R, Y, O, O, O, O, Y, R,
O, R, R, Y, Y, R, R, O
]

frames = [frame1, frame2, frame3, frame4]

while True:
    for frame in frames:
        sense.set_pixels(frame)
        time.sleep(0.5)
    sense.clear()
    time.sleep(0.2)
