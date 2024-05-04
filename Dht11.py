import Adafruit_DHT

# Set the sensor type, DHT11 or DHT22
sensor = Adafruit_DHT.DHT11

# Set the GPIO pin number where the sensor is connected
pin = 4

# Try to grab a sensor reading. Use the read_retry method which will retry up to 15 times to get a sensor reading (waiting 2 seconds between each retry).
humidity, temperature = Adafruit_DHT.read_retry(sensor, pin)

# Print the results
if humidity is not None and temperature is not None:
    print('Temperature: {0:0.1f}°C'.format(temperature))
    print('Humidity: {0:0.1f}%'.format(humidity))
else:
    print('Failed to get reading. Try again!')
