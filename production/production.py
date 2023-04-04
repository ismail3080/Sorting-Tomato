import RPi.GPIO as GPIO
import time
import shutil
import cv2
import os
import sys, getopt
import numpy as np
from edge_impulse_linux.image import ImageImpulseRunner

num_photos = 3
servo1Speed = 0.5
servo2Speed = 1
pirPin = 18  # The GPIO pin the PIR sensor is connected to
servo1 = 32
servo2 = 12
validationPercentage = 0.55 # equal 55%


# Set up GPIO mode and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo1, GPIO.OUT)
GPIO.setup(servo2, GPIO.OUT)
GPIO.setup(pirPin, GPIO.IN)
GPIO.setwarnings(False)

# Set up PWM signal on the servo GPIO pin
p = GPIO.PWM(servo1, 50)
p.start(0)
p.ChangeDutyCycle(2.5)

p2 = GPIO.PWM(servo2,50)
p2.start(0)
p2.ChangeDutyCycle(2.5)



def clear_images():
    if os.path.exists('tempimg'):
        shutil.rmtree('tempimg')
    os.mkdir('tempimg')
    
if not os.path.exists('tempimg'):
    os.makedirs('tempimg')
else:
    for filename in os.listdir('tempimg'):
        file_path = os.path.join('tempimg', filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.remove(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print('Failed to delete %s. Reason: %s' % (file_path, e))
            

def help():
    print('python classify-image.py <path_to_model.eim>')
    

def main(argv):
    try:
        opts, args = getopt.getopt(argv, "h", ["--help"])
    except getopt.GetoptError:
        help()
        sys.exit(2)

    for opt, arg in opts:
        if opt in ('-h', '--help'):
            help()
            sys.exit()
    if len(args) != 1:
        help()
        sys.exit(2)
        
    model = args[0]

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL: ' + modelfile)
    
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
            #print(timeout-time.time())
            if time.time() > timeout:
                break
            time.sleep(0.1)

        # Print a message to the console if movement was detected
        if GPIO.input(pirPin) == 1:
            print("Movement detected!")
            time.sleep(2)
            with ImageImpulseRunner(modelfile) as runner:
                try:
                    model_info = runner.init()
                    print('Loaded runner for "' + model_info['project']['owner'] + ' / ' + model_info['project']['name'] + '"')
                    labels = model_info['model_parameters']['labels']

                    # capture 3 images and save them to the tempimg directory
                    clear_images()
                    for i in range(num_photos):
                        cap = cv2.VideoCapture(0)
                        ret, frame = cap.read()
                        cv2.imwrite('tempimg/image' + str(i) + '.jpg', frame)
                        cap.release()
                    
                    imgs = []
                    for filename in os.listdir('tempimg'):
                        img = cv2.imread('tempimg/' + filename)
                        if img is None:
                            raise Exception('Failed to load image')
                        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                        imgs.append(img)
                    
                    result = []
                    for i in range(len(imgs)):
                        features , cropped = runner.get_features_from_image(imgs[i])
                        result.append(runner.classify(features))

                    data = []
                    for i in range(len(result)):
                        if("classification" in result[i]["result"].keys()):
                            for label in labels:
                                score = result[i]["result"]["classification"][label]
                                data.append(score)
                    
                    print("data: " + str(data))
                    green = 0
                    red = 0
                    for i in range(0, len(data), 2):
                        print("({0}, {1})".format(data[i], data[i+1]))
                        green += data[i]
                        red += data[i+1]
                    green_avg = round(green / 3, 2)
                    red_avg = round(red / 3, 2)
                    
                    if(red_avg > validationPercentage):
                        print("High possibility RED TOMATO")
                        print("Turning the motor for 180 deg")
                        p2.ChangeDutyCycle(12.5)
                        time.sleep(servo2Speed)
                        p2.ChangeDutyCycle(2.5)
                    #elif(green_avg > 0.55):
                    else:
                        print("High possibility GREEN TOMATO")
                        print("Turning the motor for 100 deg")
                        #Duty cycle = (100 / 180) * (12.5 - 2.5) + 2.5
                        #Duty cycle = 7.22
                        p2.ChangeDutyCycle(7.22) #7.22 duty cycle = 100 deg
                        time.sleep(servo2Speed)
                        p2.ChangeDutyCycle(2.5)
                    print("green average: " + str(green_avg))
                    print("red average: " + str(red_avg))

                finally:
                    print('here')
                    if (runner):
                        runner.stop()
            
if __name__ == "__main__":
   main(sys.argv[1:])