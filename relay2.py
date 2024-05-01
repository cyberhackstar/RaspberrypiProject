import RPi.GPIO as GPIO
import time

RELAY_PIN = 18  # Assuming the relay is connected to GPIO pin 18

# Setup GPIO mode and relay pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(RELAY_PIN, GPIO.OUT)

try:
    while True:
        print("Turning on relay")
        GPIO.output(RELAY_PIN, GPIO.HIGH)  # Turn on the relay
        time.sleep(3)  # Wait for 3 seconds
        print("Turning off relay")
        GPIO.output(RELAY_PIN, GPIO.LOW)  # Turn off the relay
        time.sleep(3)  # Wait for 3 seconds

except KeyboardInterrupt:
    GPIO.cleanup()  # Clean up GPIO on CTRL+C exit
