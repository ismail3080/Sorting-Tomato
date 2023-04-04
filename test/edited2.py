#!/usr/bin/env python


import shutil
import time
import cv2
import os
import sys, getopt
import numpy as np
from edge_impulse_linux.image import ImageImpulseRunner
import RPi.GPIO as GPIO

runner = None
num_photos = 3

green_led = 19
red_led = 21

# if there is no tempimg directory, create one
# and if there is one already, delete all the files in it

#initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(green_led, GPIO.OUT)
GPIO.setup(red_led, GPIO.OUT)
GPIO.setup(29, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.output(red_led, GPIO.HIGH)
GPIO.output(green_led, GPIO.HIGH)
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
    print('python classify-image.py <path_to_model.eim> <path_to_image.jpg>')

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

    #if len(args) != 2:
     #   help()
      #  sys.exit(2)

    model = args[0]

    dir_path = os.path.dirname(os.path.realpath(__file__))
    modelfile = os.path.join(dir_path, model)

    print('MODEL: ' + modelfile)
    while True:
        s = input('type y')
        #if GPIO.input(29) == GPIO.HIGH:
        if s == 'y':
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
                    print("green average: " + str(green_avg))
                    print("red average: " + str(red_avg))

                    if(green_avg > 0.5):
                        GPIO.output(green_led, GPIO.HIGH)
                        GPIO.output(red_led, GPIO.LOW)
                    elif(red_avg > 0.5):
                        GPIO.output(green_led, GPIO.LOW)
                        GPIO.output(red_led, GPIO.HIGH)
                    else:
                        print("no classification")

                    # elif "bounding_boxes" in res["result"].keys():
                    #     print('Found %d bounding boxes (%d ms.)' % (len(res["result"]["bounding_boxes"]), res['timing']['dsp'] + res['timing']['classification']))
                    #     for bb in res["result"]["bounding_boxes"]:
                    #         print('\t%s (%.2f): x=%d y=%d w=%d h=%d' % (bb['label'], bb['value'], bb['x'], bb['y'], bb['width'], bb['height']))
                    #         cropped = cv2.rectangle(cropped, (bb['x'], bb['y']), (bb['x'] + bb['width'], bb['y'] + bb['height']), (255, 0, 0), 1)
                    #cv2.imwrite('debug.jpg', cv2.cvtColor(cropped, cv2.COLOR_RGB2BGR))

                finally:
                    print('here')
                    if (runner):
                        runner.stop()
        else:
            print('button not pressed')
            #add delay
            time.sleep(0.2)                            

if __name__ == "__main__":
   main(sys.argv[1:])
