import RPi.GPIO as GPIO
from picamera import PiCamera
import board
import digitalio
from adafruit_bme280 import basic as adafruit_bme280
import pickle
import time
from time import sleep
import math
import cv2
print("worked")
import numpy as np
import time
import board
import busio
import adafruit_gps
import serial
import adafruit_rfm9x
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.output(17, GPIO.HIGH)
sleep(0.25) #might have to increase this to 2
GPIO.output(17, GPIO.LOW)
sleep(0.25)

with open("/home/pi/Downloads/gps_topleftcoords.pkl", "rb") as pf:
    topleftcoords = pickle.load(pf)
print("loaded")
print(topleftcoords[0])

with open(r"/home/pi/Downloads/discriptors.pkl", "rb") as fp:
    descriptorslist = pickle.load(fp)
print(len(descriptorslist))

print(len(descriptorslist))
print(len(descriptorslist[0]), len(descriptorslist[2]))

import adafruit_rfm9x
from digitalio import DigitalInOut, Direction, Pull

i2c = busio.I2C(board.SCL, board.SDA)

CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 433.0)
rfm9x.tx_powerr = 23
#rfm9x.send() or rfm.9x.receive()

uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b"PMTK220,1000")
#gps test
last_print = time.monotonic()
while True:
    #print("gps is searching")
    gps.update()
    # Every second print out current location details if there's a fix.
    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        if gps.has_fix:
            # Try again if we don't have a fix yet.
            #print("Waiting for fix...")
            break
print("Latitude: {0:.6f} degrees".format(gps.latitude))
print("Longitude: {0:.6f} degrees".format(gps.longitude))
print("Fix quality: {}".format(gps.fix_quality))

GPIO.output(17, GPIO.HIGH)
sleep(0.5) #might have to increase this to 2
GPIO.output(17, GPIO.LOW)
sleep(0.5)
#GPIO.output(17, GPIO.HIGH)
#sleep(2) #might have to increase this to 2
#GPIO.output(17, GPIO.LOW)
#sleep(2)
#print("led")

#print(len(loaded_list))
x_incr = (-117.809809 + 117.808915)
y_incr = (35.347124 - 35.346395)

def contains_blue(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #range for blue 
    hsv_l = np.array([100, 150, 0])
    hsv_h = np.array([140, 255, 255])
    # Find blue pixels in the image
    if np.count_nonzero(cv2.inRange(hsv, hsv_l, hsv_h)) > 200:
        return True
    else:
        return False
print("containsblue with cv2 cmd")
img_list = []


#GPIO.output(17, GPIO.HIGH)
#sleep(2)
#GPIO.output(17, GPIO.LOW)
#sleep(2)
try:
    sift = cv2.xfeatures2d.SIFT_create()
    print("siftworked")
    camera=PiCamera()
    #camera.capture("/home/pi/Downloads/camtest/try.jpg")
    print("camera")
    
    start_time = time.time()
    
    i2c = board.I2C()
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c, 0x76)
    #bme_cs = digitalio.DigitalInOut(board.D10)
    #bme280 = adafruit_bme280.Adafruit_BME280_SPI(spi, bme_cs) #sea level pressure at FAR is 29.34Hg which is 993.566hPA
    bme280.sea_level_pressure = 993.566
    altitudes_list = []
    img_no = 0
    #add some conditional regarding if the button has been pressed to encompass all this

    init_alt = 0
    for i in range(100):
        calib_alt = bme280.altitude  #bme280 only works with absolute altitude, so we'll establish a baseline on startup at ground
        init_alt+=calib_alt
    init_alt = init_alt/100
    #print(init_alt)
    match = 0
    checkPass = 0 
    while True:
        check_alt = bme280.altitude
        print("after")
        for i in range(5):
            rfm9x.send(bytes("check_alt{}".format(check_alt-init_alt), "utf-8"))
        print("check_alt")
        #GPIO.output(17, GPIO.HIGH)
        #sleep(0.25) #might have to increase this to 2
        #GPIO.output(17, GPIO.LOW)
        #sleep(0.25)
        print("blink")
        #check_alt = 9000 #comment this out

        if (check_alt-init_alt) > 200: #checks if it has gone high, no point taking pictures the way up   
            checkPass = 1
            check_alt = bme280.altitude
            #check_alt=0 #comment this out
        if check_alt-init_alt < 150 and checkPass==1: #or whatever meters below which we should start taking photos
            #check_alt = 10000 #comment this out
            while check_alt-init_alt>20:
                rfm9x.send(bytes("pic", "utf-8"))
                #GPIO.output(17, GPIO.HIGH)
                #sleep(0.02) #might have to increase this to 2
                #GPIO.output(17, GPIO.LOW)
                current_time = time.time()-start_time
                #camera.capture("/home/pi/Downloads/subscale_test_imgs/img_"+str(img_no)+ "alt: "+str(round(check_alt-init_alt, 4))+"time: "+str(round(current_time,5))+".jpg") #take a picture, for CDR we can probably just save these but we need to get numpy working for analysis
                camera.resolution = (640, 480)
                camera.framerate = 24
                im = np.empty((640,480,3), dtype = np.uint8)
                camera.capture(im, "rgb")
                #avoid processing blue
                if contains_blue(im):
                    continue
                else:
                    img_list.append(im)
                img_no+=1
                altitudes_list.append(["alt"+str(check_alt), "time: "+ str(current_time)])
                check_alt = bme280.altitude
                #check_alt = 1000
            break
    #processing
    guesses = []
    acc_count = 0
    #sift = cv2.SIFT_create()
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    
    gps.update()
    ylat = gps.latitude
    xlong = gps.longitude
    firstsqr = 0
    for coords in topleftcoords:
        firstsqr+=1
        if (ylat <= coords[0] and ylat >= (coords[0]- y_incr)):
            if (xlong <= coords[1] and xlong >= coords[1]+x_incr):
                break
    rfm9x.send(bytes("{} {} {} {}".format(2, ylat, xlong, firstsqr),"utf-8"))
    
    etc_counter = 0
    for i in range(len(img_list)//3):  #for each pi cam image
        test_keypoints, test_descriptors = sift.detectAndCompute(img_list[i],None)      #calculate descriptor of that image
        etc_counter+=1
        matches_counts = []
        if test_descriptors is None:
            print("no descriptor")
            continue
        for j in descriptorslist:                      #compare that discriptor with every descriptor of the 126 classifying images
            #matches = bf.match(j,test_descriptors)  
            matches = flann.knnMatch(j,test_descriptors,k=2)  
            good = []
            for m,n in matches:
                if m.distance < 0.6*n.distance:
                    good.append(m)
            matches_counts.append(len(good))        

        guess = np.argmax(matches_counts)              #return which classifying image most matched the test image
        guesses.append(guess)

    final_mode = stats.mode(guesses)
    final_guess = final_mode[0][0]
    #print(final_guess)
    gps.update()
    ylat = gps.latitude
    xlong = gps.longitude 
    firstsqr = 0
    for coords in topleftcoords:
        firstsqr+=1
        if (ylat <= coords[0] and ylat >= (coords[0]- y_incr)):
            if (xlong <= coords[1] and xlong >= coords[1]+x_incr):
                break
    #print(b)
    if firstsqr == final_guess:
        match = 1
    
    if match == 1:
        with open(r"/home/pi/Downloads/backup/backupresult", "w") as f:
            f.write("{} {} {} {}".format(1, ylat, xlong, firstsqr))
        while True:
            rfm9x.send(bytes("{} {} {} {}".format(1, ylat, xlong, firstsqr),"utf-8"))
    else:
        with open(r"/home/pi/Downloads/backup/backupresult", "w") as f:
            f.write("{} {} {} {}".format(0, ylat, xlong, firstsqr))
        while True:
            rfm9x.send(bytes("{} {} {} {}".format(0, ylat, xlong, firstsqr)), "utf-8")
    
    for i in range(len(img_list) - etc_counter):  #for each pi cam image
        test_keypoints, test_descriptors = sift.detectAndCompute(img_list[i+etc_counter],None)      #calculate descriptor of that image
        matches_counts = []
        if test_descriptors is None:
            print("no descriptor")
            continue
        for j in descriptorslist:                      #compare that discriptor with every descriptor of the 126 classifying images
            #matches = bf.match(j,test_descriptors)  
            matches = flann.knnMatch(j,test_descriptors,k=2)  
            good = []
            for m,n in matches:
                if m.distance < 0.6*n.distance:
                    good.append(m)
            matches_counts.append(len(good))        

        guess = np.argmax(matches_counts)              #return which classifying image most matched the test image
        guesses.append(guess)

    final_mode = stats.mode(guesses)
    final_guess = final_mode[0][0]
    #print(final_guess)
    gps.update()
    ylat = gps.latitude
    xlong = gps.longitude 
    firstsqr = 0
    for coords in topleftcoords:
        firstsqr+=1
        if (ylat <= coords[0] and ylat >= (coords[0]- y_incr)):
            if (xlong <= coords[1] and xlong >= coords[1]+x_incr):
                break
    #print(b)
    if firstsqr == final_guess:
        match = 1
    
    if match == 1:
        with open(r"/home/pi/Downloads/backup/backupresult", "w") as f:
            f.write("{} {} {} {}".format(1, ylat, xlong, firstsqr))
        while True:
            rfm9x.send(bytes("{} {} {} {}".format(1, ylat, xlong, firstsqr),"utf-8"))
    else:
        with open(r"/home/pi/Downloads/backup/backupresult", "w") as f:
            f.write("{} {} {} {}".format(1, ylat, xlong, firstsqr))
        while True:
            rfm9x.send(bytes("{} {} {} {}".format(0, ylat, xlong, firstsqr)), "utf-8")
    
    with open("altitudes_list", "wb") as fp:
        pickle.dump(altitudes_list,fp)
    with open("altitudes_list", "rb") as fp:
        b=pickle.load(fp)

#so we can look at altitudes corresponding to img#
    
except:
    print("f")
    #for i in range(230):
        #rfm9x.send(bytes("failed", "utf-8"))
    sleep(230)
    #for i in range(2000):
        #print("picture-taking rn")
        #img_no = i
        #sleep(2)
        #camera.capture("/home/pi/Downloads/subscale_test_imgs/img_%s.jpg" % img_no)
    gps.update()
    ylat = gps.latitude
    xlong = gps.longitude
    print(lat, long)
    sqr = 0
    for coords in topleftcoords:
        sqr+=1
        if (ylat <= coords[0] and ylat >= (coords[0]- y_incr)):
            if (xlong <= coords[1] and xlong >= coords[1]+x_incr):
                break
    with open(r"/home/pi/Downloads/backup/backupresult", "w") as f:
            f.write("{} {} {} {}".format(0, ylat, xlong, firstsqr))
    while True:
        rfm9x.send(bytes("{} {} {} {}".format(0, ylat, xlong, sqr), "utf-8"))