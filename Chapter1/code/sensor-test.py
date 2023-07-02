from sense_hat import SenseHat

sense = SenseHat()
temperature = sense.get_temperature()
humidity = sense.get_humidity()
pressure = sense.get_pressure()
accelerometer = sense.get_accelerometer_raw()
gyroscope = sense.get_gyroscope_raw()

print("Temperature: {:.2f}°C".format(temperature))
print("Humidity: {:.2f}%".format(humidity))
print("Pressure: {:.2f} millibars".format(pressure))

print("Accelerometer Data: x={:.2f}, y={:.2f}, z={:.2f}".format(\
    accelerometer['x'], accelerometer['y'], accelerometer['z']))

print("Gyroscope Data: x={:.2f}, y={:.2f}, z={:.2f}".format(\
    gyroscope['x'], gyroscope['y'], gyroscope['z']))
