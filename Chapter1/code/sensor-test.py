from sense_hat import SenseHat

sense_hat = SenseHat()
temp = sense_hat.get_temperature()
humidity = sense_hat.get_humidity()
press = sense_hat.get_pressure()
accel = sense_hat.get_accelerometer_raw()
gyroscope = sense_hat.get_gyroscope_raw()

print("Temperature: {:.2f}°C".format(temp))
print("Humidity: {:.2f}%".format(humidity))
print("Pressure: {:.2f} millibars".format(press))

print("Accelerometer Data: x={:.2f}, y={:.2f},  z={:.2f}".format(accel['x'], accel['y'], accel['z']))
print("Gyroscope Data: x={:.2f}, y={:.2f}, z={:.2f}".format(gyroscope['x'], gyroscope['y'], gyroscope['z']))

