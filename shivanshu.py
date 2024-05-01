import RPi.GPIO as GPIO
import bmpsensor
import Adafruit_DHT
import time
from gpiozero import Buzzer
from BlynkLib import Blynk
from BlynkTimer import BlynkTimer

# Initialize GPIO and sensors
DHT_SENSOR = Adafruit_DHT.DHT22
DHT_PIN = 4
servoPIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
p = GPIO.PWM(servoPIN, 50)  # GPIO 17 for PWM with 50Hz
p.start(2.5)  # Initialization
buzzer = Buzzer(23)


BLYNK_AUTH_TOKEN = 'YourAuthToken'
blynk = Blynk(BLYNK_AUTH_TOKEN)
timer = BlynkTimer()


def send_dht_data():
    humidity, temperature = Adafruit_DHT.read(DHT_SENSOR, DHT_PIN)
    if humidity is not None and temperature is not None:
        print("Temp={0:0.1f}C Humidity={1:0.1f}%".format(temperature, humidity))
        blynk.virtual_write(0, temperature)
        blynk.virtual_write(1, humidity)
    else:
        print("DHT Sensor failure. Check wiring.")


def send_bmp_data():
    temp, pressure, altitude = bmpsensor.readBmp180()
    if temp is not None and pressure is not None and altitude is not None:
        print("Temperature is ", temp)  # degC
        print("Pressure is ", pressure)  # Pressure in Pa
        print("Altitude is ", altitude)  # Altitude in meters
        blynk.virtual_write(2, temp)
        blynk.virtual_write(3, pressure)
        blynk.virtual_write(4, altitude)
    else:
        print("BMP Sensor failure. Check wiring.")

# Set up Blynk connection
@blynk.on("connected")
def blynk_connected():
    print("Connected to Blynk")


def control_servo_and_buzzer():
    humidity, temperature = Adafruit_DHT.read(DHT_SENSOR, DHT_PIN)
    temp, pressure, altitude = bmpsensor.readBmp180()

    if humidity is not None and temperature is not None and temp is not None and pressure is not None and altitude is not None:
        if temperature < 29 and pressure < 98000:
            p.ChangeDutyCycle(0)
            time.sleep(0.5)
            buzzer.on()
            time.sleep(1)
        else:
            p.ChangeDutyCycle(9.0)
            time.sleep(0.5)
            buzzer.off()
    else:
        print("Sensor failure. Check wiring.")

timer.set_interval(2, send_dht_data)
timer.set_interval(2, send_bmp_data)
timer.set_interval(2, control_servo_and_buzzer)

while True:
    blynk.run()
    timer.run()
