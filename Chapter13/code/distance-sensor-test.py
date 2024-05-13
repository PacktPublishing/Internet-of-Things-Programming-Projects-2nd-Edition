import time
import board
import busio
import adafruit_vl53l0x

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create VL53L0X object
vl53 = adafruit_vl53l0x.VL53L0X(i2c)

while True:
    print('Distance: {0}mm'.format(vl53.range))
    time.sleep(1)
