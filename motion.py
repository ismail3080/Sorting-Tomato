import RPi.GPIO as GPIO
import time

pir_sensor = 7
led = 11
buzz = 13

GPIO.setmode(GPIO.BOARD)

GPIO.setup(led,GPIO.OUT)
GPIO.setup(buzz,GPIO.OUT)

GPIO.setup(pir_sensor, GPIO.IN)

current_state = 0

# turn off the led when motion is detected

try:
    while True:
        time.sleep(0.1)
        current_state = GPIO.input(pir_sensor)
        if current_state == 1:
            GPIO.output(led, GPIO.HIGH)
            # let the buzzer beep with low frequency
            
            GPIO.output(buzz, GPIO.HIGH)
        else:
            GPIO.output(led, GPIO.LOW)
            GPIO.output(buzz, GPIO.LOW)

except KeyboardInterrupt:
    GPIO.cleanup()



