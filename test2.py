import RPi.GPIO as GPIO
import time

servo1Speed = 0.5
pirPin = 18  # The GPIO pin the PIR sensor is connected to

# Set up GPIO mode and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.OUT)
GPIO.setup(pirPin, GPIO.IN)

# Set up PWM signal on the servo GPIO pin
p = GPIO.PWM(32, 50)
p.start(0)
p.ChangeDutyCycle(2.5)


try:
    while True:
        # Move the servo to 90 degrees
        p.ChangeDutyCycle(7.5)
        print("Going UP")
        time.sleep(servo1Speed)
        # Move the servo to 0 degrees
        p.ChangeDutyCycle(2.5)
        print("Going down")
        time.sleep(2)

        # Wait for movement to be detected by the PIR sensor
        timeout = time.time() + 10  # Set a timeout of 10 seconds
        while GPIO.input(pirPin) == 0:
            print(time.time()-timeout)
            if time.time() > timeout:
                break
            time.sleep(0.1)

        # Print a message to the console if movement was detected
        if GPIO.input(pirPin) == 1:
            print("Movement detected!")
            time.sleep(2)
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
