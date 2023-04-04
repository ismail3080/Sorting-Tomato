# test servo motor connected to raspberry pi 4 GPIO pin 18

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)

p = GPIO.PWM(18, 50) # GPIO 18 for PWM with 50Hz
p.start(2.5) # Initialization

try:
    while True:
        p.ChangeDutyCycle(2.5) # turn towards 0 degree
        time.sleep(1) # sleep 1 second
        p.ChangeDutyCycle(12.5) # turn towards 180 degree
        time.sleep(1) # sleep 1 second
        p.ChangeDutyCycle(7.5) # turn towards 90 degree
        time.sleep(1) # sleep 1 second
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()


