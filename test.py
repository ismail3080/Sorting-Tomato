import RPi.GPIO as GPIO
import time
servo1Speed = 0.5

# Set up GPIO mode and pin
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)

# Set up PWM signal on the GPIO pin
p = GPIO.PWM(12, 50)
p.start(0)
while True:
    print("Going to 0 deg")
    p.ChangeDutyCycle(2.5)
    time.sleep(4)
    print("Going to 180")
    p.ChangeDutyCycle(12.5)
    time.sleep(4)
    


