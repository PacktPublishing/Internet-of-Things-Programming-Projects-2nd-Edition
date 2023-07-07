from sense_emu import SenseHat
import time

class RedXAnimation:
    black = (0, 0, 0)
    red = (255, 0, 0)

    frame1 = [
        red, black, black, black, black, black, black, red,
        black, red, black, black, black, black, red, black,
        black, black, red, black, black, red, black, black,
        black, black, black, red, red, black, black, black,
        black, black, black, red, red, black, black, black,
        black, black, red, black, black, red, black, black,
        black, red, black, black, black, black, red, black,
        red, black, black, black, black, black, black, red
    ]

    frame2 = [
        red, red, red, red, red, red, red, red,
        red, red, red, red, red, red, red, red,
        red, red, red, red, red, red, red, red,
        red, red, red, red, red, red, red, red,
        red, red, red, red, red, red, red, red,
        red, red, red, red, red, red, red, red,
        red, red, red, red, red, red, red, red,
        red, red, red, red, red, red, red, red
    ]

    def __init__(self, rotation=0):
        self.sense = SenseHat()
        self.sense.set_rotation(rotation)

    def display_animation(self, duration):
        num_frames = 2
        frame_duration = duration / num_frames

        start_time = time.time()
        end_time = start_time + 59

        while time.time() < end_time:
            for frame in [self.frame1, self.frame2]:
                self.sense.set_pixels(frame)
                time.sleep(frame_duration)

if __name__ == "__main__":
    animation = RedXAnimation(rotation=270)
    animation.display_animation(duration=1)
 