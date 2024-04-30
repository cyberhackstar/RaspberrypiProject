import BlynkLib
import RPi.GPIO as GPIO
from BlynkTimer import BlynkTimer
from numpy import interp    # To scale values
import time
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from gpiozero import Buzzer
from gpiozero import OutputDevice
import Adafruit_DHT

GPIO.setmode(GPIO.BCM)

DHT_SENSOR = Adafruit_DHT.DHT11
DHT_PIN = 4

ULTRASONIC_TRIG_PIN = 23
ULTRASONIC_ECHO_PIN = 24

RELAY_PIN = 18  # Assuming the relay is connected to GPIO pin 18
BUZZER_PIN = 25  # Assuming the buzzer is connected to GPIO pin 25

# Initialize the buzzer
buzzer = Buzzer(BUZZER_PIN)
relay = OutputDevice(RELAY_PIN, active_high=False, initial_value=False)

BLYNK_AUTH_TOKEN = 'jsKFcrHy1qGYrUfFICjM4QcyxtYiYp6N'

# Initialize Blynk
blynk = BlynkLib.Blynk(BLYNK_AUTH_TOKEN)

# Create BlynkTimer Instance
timer = BlynkTimer()

# Setup ultrasonic sensor GPIO pins
GPIO.setup(ULTRASONIC_TRIG_PIN, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO_PIN, GPIO.IN)

# Setup relay and buzzer pins
#GPIO.setup(RELAY_PIN, GPIO.OUT)
#GPIO.output(RELAY_PIN, GPIO.LOW)  # Initially, turn off the relay

#GPIO.setup(BUZZER_PIN, GPIO.OUT)
#GPIO.output(BUZZER_PIN, GPIO.LOW)  # Initially, turn off the buzzer

# Initialize the I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
 
# Create an ADS1115 object
ads = ADS.ADS1115(i2c)
 
# Define the analog input channel
channel = AnalogIn(ads, ADS.P0)

# Function to sync the data from virtual pins
@blynk.on("connected")
def blynk_connected():
    print("Hi, You have Connected to New Blynk")
    time.sleep(2)

# Function to control relay based on soil moisture
def control_relay(soil_moisture_percentage):
    if soil_moisture_percentage < 30:
        print("Soil moisture is high. Turning on relay.")
        #GPIO.output(RELAY_PIN, GPIO.HIGH)  # Turn on the relay
        # Update Blynk label to show "Pump On"
        relay.on()
        blynk.virtual_write(4, "Pump On")
    else:
        print("Soil moisture is low. Turning off relay.")
        GPIO.output(RELAY_PIN, GPIO.LOW)  # Turn off the relay
        # Update Blynk label to show "Pump Off"
        relay.off()
        blynk.virtual_write(4, "Pump Off")


# Function to send notification to Blynk app
def send_notification():
    #print("Sending notification: Empty_Tank")
    #blynk.log_event("Empty_Tank", "Tank is empty!")
    blynk.send_internal(0, "notify", "Tank is empty!")

# Function to collect data from sensors & send it to Server
def myData():
    # Read DHT11 sensor data
    humidity, temperature = Adafruit_DHT.read(DHT_SENSOR, DHT_PIN)
    if humidity is not None and temperature is not None:
        print("Temp={0:0.1f}C Humidity={1:0.1f}%".format(temperature, humidity))
    else:
        print("DHT11 Sensor failure. Check wiring.")

    # Read ultrasonic sensor data
    GPIO.output(ULTRASONIC_TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(ULTRASONIC_TRIG_PIN, False)
    
    pulse_start = time.time()
    pulse_end = time.time()
    
    while GPIO.input(ULTRASONIC_ECHO_PIN) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ULTRASONIC_ECHO_PIN) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    print("Distance: ", distance, "cm")
    inverted_distance = 10 - distance
    distance_percentage = (inverted_distance / 10) * 100

    # Read soil moisture sensor data
    raw_soil_moisture = interp(channel.value, [0, 65535], [0, 100])
    print("Raw Soil Moisture:", raw_soil_moisture)

    # Invert soil moisture readings
    inverted_soil_moisture = 100 - raw_soil_moisture
    # Map inverted readings to a percentage scale
    soil_moisture_percentage = ((inverted_soil_moisture - 0) / (100 - 0)) * 100
    print("Soil Moisture Percentage:", soil_moisture_percentage)

    # Send data to Blynk app
    blynk.virtual_write(0, temperature)
    blynk.virtual_write(1, humidity)
    blynk.virtual_write(2, distance_percentage)
    blynk.virtual_write(3, soil_moisture_percentage)
    print("Values sent to New Blynk Server!")
    
    # Control relay based on soil moisture
    control_relay(soil_moisture_percentage)
    
    # Turn on buzzer and send notification if distance exceeds 10cm
    if distance > 10:
        print("Tank is Empty. Turning on buzzer.")
        #GPIO.output(BUZZER_PIN, GPIO.HIGH)  # Turn on the buzzer
        # Send notification using log_event
        buzzer.on()
        send_notification()
    else:
        buzzer.off()
        #GPIO.output(BUZZER_PIN, GPIO.LOW)  # Turn off the buzzer

timer.set_interval(2, myData)

while True:
    blynk.run()
    timer.run()


