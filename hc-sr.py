import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

PIR_SENSOR_PIN = 16
GPIO.setup(PIR_SENSOR_PIN, GPIO.IN)

print("Lancement du capteur de presence IR (CTRL-C pour quitter)")
time.sleep(2)

try:
	while True:
		if GPIO.input(PIR_SENSOR_PIN):
			print("Mouvement detecte!")
		time.sleep(1)

except KeyboardInterrupt:
	# If there is a KeyboardInterrupt (when you press ctrl+c), exit the program and cleanup
	print("Cleaning up!")
	# Reinitialisation des parametres GPIOs
	GPIO.cleanup()

