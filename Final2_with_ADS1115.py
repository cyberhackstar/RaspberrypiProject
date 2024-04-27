import BlynkLib
import RPi.GPIO as GPIO
from BlynkTimer import BlynkTimer
import Adafruit_ADS1x15
from time import sleep

DHT_SENSOR = Adafruit_DHT.DHT11
DHT_PIN = 4

ULTRASONIC_TRIG_PIN = 23
ULTRASONIC_ECHO_PIN = 24

RELAY_PIN = 18  # Assuming the relay is connected to GPIO pin 18
BUZZER_PIN = 25  # Assuming the buzzer is connected to GPIO pin 25

# Set GPIO pin numbering mode
GPIO.setmode(GPIO.BCM)  # or GPIO.setmode(GPIO.BOARD) for physical pin numbering

BLYNK_AUTH_TOKEN = 'jsfpl_ACPJIVDdkFJ9THE5PkCVvYR8FS'

# Initialize Blynk
blynk = BlynkLib.Blynk(BLYNK_AUTH_ token)

# Create BlynkTimer Instance
timer = BlynkTimer()

# Setup ultrasonic sensor GPIO pins
GPIO.setup(ULTRASONIC_TRIG_PIN, GPIO.OUT)
GPIO.setup(ULTRASONIC_ECHO_PIN, GPIO.IN)

# Setup relay and buzzer pins
GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.LOW)  # Initially, turn off the relay

GPIO.setup(BUZZER_PIN, GPIO.OUT)
GPIO.output(BUZZER_PIN, GPIO.LOW)  # Initially, turn off the buzzer

# Initialize ADS1115 with default address (0x48) and gain of 1
adc = Adafruit_ADS1x15.ADS1115()

# Function to sync the data from virtual pins
@blynk.on("connected")
def blynk_connected():
    print("Hi, You have Connected to New Blynk")
    sleep(2)


# Function to control relay based on soil moisture
def control_relay(soil_moisture):
    if soil_moisture > 60:
        print("Soil moisture is high. Turning off relay.")
        GPIO.output(RELAY_PIN, GPIO.LOW)  # Turn off the relay
    else:
        print("Soil moisture is low. Turning on relay.")
        GPIO.output(RELAY_PIN, GPIO.HIGH)  # Turn on the relay


# Function to send notification to Blynk app
def send_notification():
    print("Sending notification: Empty_Tank")
    blynk.log_event("Empty_Tank", "Tank is empty!")


# Function to collect data from sensors & send it to Server
def myData():
    # Read DHT11 sensor data
    humidity, temperature = Adafruit_DHT.read(DHT_SENSOR, DHT_PIN)
    if humidity is not None and temperature is not None:
        print("Temp={0:0.1f}C Humidity={1:0.1f}%".format(temperature, humidity))
    else:
        print("DHT11 Sensor failure. Check wiring.")

    # Read ultrasonic sensor data (same as before)
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

    # Read soil moisture sensor data using ADS1115
    value = adc.read_adc(0, gain=1)  # Read from channel 0 with gain 1
    voltage = value / 32767 * 3.3  # Convert ADC value to voltage (assuming full range is 3.3V)
    # Map the voltage reading to a moisture value based on your sensor (adjust as needed)
    soil_moisture = 100 - (voltage * 100) / 3.3

    print("Soil Moisture:", soil_moisture)

    # Send data to Blynk app
    blynk.virtual_write(1, humidity)
    blynk.virtual_write(0, temperature)
    blynk.virtual_write(2, distance)
    blynk.virtual_write(3, soil_moisture)
    print("Values sent to New Blynk Server!")

    # Control relay based on soil moisture
    control_relay(soil_moisture)

    # Turn on buzzer and send notification if distance exceeds 10cm (same as before)
    if distance > 10:
        print("Distance is greater than 10cm. Turning on buzzer and sending notification.")
        GPIO.output(BUZZER_PIN, GPIO.HIGH)  # Turn on the buzzer
        # Send notification using log_event
        send_notification()
    else:
        GPIO.output(BUZZER_PIN, GPIO.LOW)  # Turn off the buzzer


timer.set_interval(2, myData)

while True:
    blynk.run()
    timer.run()
