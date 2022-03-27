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

try:
    import logging
    import sys
    import time
    from Adafruit_BNO055 import BNO055

    # Enable verbose debug logging if -v is passed as a parameter.
    if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
        logging.basicConfig(level=logging.DEBUG)

    # Initialize the BNO055 and stop if something went wrong.
    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

    # Print system status and self test result.
    status, self_test, error = bno.get_system_status()
    print('System status: {0}'.format(status))
    print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
    # Print out an error if system status is in error mode.
    if status == 0x01:
        print('System error: {0}'.format(error))
        print('See datasheet section 4.3.59 for the meaning.')

    # Print BNO055 software revision and other diagnostic data.
    sw, bl, accel, mag, gyro = bno.get_revision()
    print('Software version:   {0}'.format(sw))
    print('Bootloader version: {0}'.format(bl))
    print('Accelerometer ID:   0x{0:02X}'.format(accel))
    print('Magnetometer ID:    0x{0:02X}'.format(mag))
    print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))

    print('Reading BNO055 data, press Ctrl-C to quit...')
except:
    pass    

x_incr =(117.808914-117.809808)
y_incr =(35.346394-35.347125)
start = [32.846399, -115.273347]
newlist = []
newlist.append(start)
for i in range(400):
    if i%20 == 19 and i!=0:
        #print(len(newlist))
        prev = newlist[len(newlist)-20]
        add = [prev[0]+y_incr,prev[1]]
        newlist.append(add) 
    else: 
        prev = newlist[-1]
        add = [prev[0],prev[1]-x_incr]
        newlist.append(add)
topleftcoords = newlist
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

uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart, debug=False)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b"PMTK220,1000")
#gps test
last_print = time.monotonic()
while True:
    #print("gps is searching")
    gps.update()
    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        if gps.has_fix:
            break
print("Latitude: {0:.6f} degrees".format(gps.latitude))
print("Longitude: {0:.6f} degrees".format(gps.longitude))
print("Fix quality: {}".format(gps.fix_quality))
for i in range(5):
    rfm9x.send(bytes("GPS Found", "utf-8"))

def contains_blue(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #range for blue 
    hsv_l = np.array([100, 150, 0])
    hsv_h = np.array([140, 255, 255])
    #find blue pixels in image
    if np.count_nonzero(cv2.inRange(hsv, hsv_l, hsv_h)) > 200:
        return True
    else:
        return False
print("containsblue with cv2 cmd")
img_list = []

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
    #bme280 = adafruit_bme280.Adafruit_BME280_SPI(spi, bme_cs) 
    #sea level pressure at FAR is 29.34Hg which is 993.566hPA
    bme280.sea_level_pressure = 993.566
    altitudes_list = []
    img_no = 0

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
        altitudes_list.append(check_alt-init_alt)
        #print("after")
        write_alts = open("write_alts.txt", "w")
        write_alts.writelines("{}".format((check_alt-init_alt)))
        write_alts.close()
        for i in range(5):
            rfm9x.send(bytes("check_alt{}".format(check_alt-init_alt), "utf-8"))
        print("check_alt")

        if (check_alt-init_alt) > 200: #checks if it has gone high, no point taking pictures the way up   
            checkPass = 1
            check_alt = bme280.altitude
            #check_alt=0 #comment this out
        if check_alt-init_alt < 145 and checkPass==1: #or whatever meters below which we should start taking photos
            write_alts = open("status.txt", "w") #NEED TO MAKE THIS EMPTY TXT FILE IN THE PAYLOAD BEFOREHAND
            write_alts.writelines("pics")
            write_alts.close()
            while check_alt-init_alt>20:
                rfm9x.send(bytes("pic", "utf-8"))
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
    write_alts = open("status.txt", "w") #NEED TO MAKE THIS EMPTY TXT FILE IN THE PAYLOAD BEFOREHAND
    write_alts.writelines("past the loop")
    write_alts.close()
    guesses = []
    acc_count = 0
    #sift = cv2.SIFT_create()
    FLANN_INDEX_KDTREE = 1
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks = 50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    while True:
        gps.update()
        if gps.has_fix:
            break
    ylat = gps.latitude
    xlong = gps.longitude
    firstsqr = -1
    for coords in topleftcoords:
        firstsqr+=1
        if (ylat <= coords[0] and ylat >= (coords[0]+ y_incr)):
            if (xlong <= coords[1] and xlong >= coords[1]+x_incr):
                break
    
    write_alts = open("status.txt", "w") #NEED TO MAKE THIS EMPTY TXT FILE IN THE PAYLOAD BEFOREHAND
    write_alts.writelines("past stg1 processing")
    write_alts.writelines("{} {} {} {}".format(match, ylat, xlong, firstsqr))
    write_alts.close()
    for i in range(5):
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
    firstsqr = -1
    for coords in topleftcoords:
        firstsqr+=1
        if (ylat <= coords[0] and ylat >= (coords[0]+ y_incr)):
            if (xlong <= coords[1] and xlong >= coords[1]+x_incr):
                break
    #print(b)
    if firstsqr == final_guess:
        match = 1
    
    if match == 1:
        with open(r"/home/pi/Downloads/backup/backupresult", "w") as f:
            f.write("{} {} {} {}".format(match, ylat, xlong, firstsqr))
        with open("altitudes_list", "wb") as fp:
            pickle.dump(altitudes_list,fp)
        for i in range(15):
            rfm9x.send(bytes("{} {} {} {}".format(match, ylat, xlong, firstsqr),"utf-8"))
    else:
        with open(r"/home/pi/Downloads/backup/backupresult", "w") as f:
            f.write("{} {} {} {}".format(match, ylat, xlong, firstsqr))
        with open("altitudes_list", "wb") as fp:
            pickle.dump(altitudes_list,fp)
        for i in range(15):
            rfm9x.send(bytes("{} {} {} {}".format(match, ylat, xlong, firstsqr)), "utf-8")
    
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
    firstsqr = -1
    for coords in topleftcoords:
        firstsqr+=1
        if (ylat <= coords[0] and ylat >= (coords[0]+ y_incr)):
            if (xlong <= coords[1] and xlong >= coords[1]+x_incr):
                break
    #print(b)
    if firstsqr == final_guess:
        match = 1
    
    if match == 1:
        with open(r"/home/pi/Downloads/backup/backupresult", "w") as f:
            f.write("{} {} {} {}".format(match, ylat, xlong, firstsqr))
        while True:
            rfm9x.send(bytes("{} {} {} {}".format(match, ylat, xlong, firstsqr),"utf-8"))
    else:
        with open(r"/home/pi/Downloads/backup/backupresult", "w") as f:
            f.write("{} {} {} {}".format(match, ylat, xlong, firstsqr))
        while True:
            rfm9x.send(bytes("{} {} {} {}".format(match, ylat, xlong, firstsqr)), "utf-8")
    
    with open("altitudes_list", "wb") as fp:
        pickle.dump(altitudes_list,fp)
    with open("altitudes_list", "rb") as fp:
        b=pickle.load(fp)

#so we can look at altitudes corresponding to img#
    
except:
    sleep(230)
    while True:
        gps.update()
        if gps.has_fix:
            break
    ylat = gps.latitude
    xlong = gps.longitude
    print(lat, long)
    sqr = -1
    for coords in topleftcoords:
        sqr+=1
        if (ylat <= coords[0] and ylat >= (coords[0]+ y_incr)):
            if (xlong <= coords[1] and xlong >= coords[1]+x_incr):
                break
    with open(r"/home/pi/Downloads/backup/backupresult", "w") as f:
            f.write("{} {} {} {}".format(match, ylat, xlong, firstsqr))
    while True:
        rfm9x.send(bytes("{} {} {} {}".format(match, ylat, xlong, sqr), "utf-8"))
