from servo import Servo
import utime

servo = Servo(14)
servo.set_position(0)
utime.sleep(1)
servo.set_position(90)
utime.sleep(1)
servo.set_position(180)
utime.sleep(1)

# Return servo to initial position
servo.set_position(0)