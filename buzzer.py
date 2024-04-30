import RPi.GPIO as GPIO
import time

# Set GPIO mode (BCM or BOARD)
GPIO.setmode(GPIO.BCM)

# Set the GPIO pin connected to the buzzer (replace with your actual pin)
buzzer_pin = 18

# Setup the GPIO pin as output
GPIO.setup(buzzer_pin, GPIO.OUT)

try:
  # Loop forever (you can modify this for a specific number of cycles)
  while True:
    # Turn the buzzer on
    GPIO.output(buzzer_pin, GPIO.HIGH)
    print("Buzzer ON")
    time.sleep(2)  # Wait for 2 seconds

    # Turn the buzzer off
    GPIO.output(buzzer_pin, GPIO.LOW)
    print("Buzzer OFF")
    time.sleep(2)  # Wait for 2 seconds

# Exit handler (optional)
except KeyboardInterrupt:
  # Cleanup GPIO on exit
  GPIO.cleanup()
  print("Exiting program")
