import RPi.GPIO as GPIO
from picamera import PiCamera
import board
import digitalio
from adafruit_bme280 import basic as adafruit_bme280
import pickle
import time
from time import sleep
import math

camera=PiCamera()
start_time = time.time()

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.output(17, GPIO.HIGH)
sleep(2) #might have to increase this to 2
GPIO.output(17, GPIO.LOW)
sleep(2)
try:
    i2c = board.I2C()
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
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
    print(init_alt)
                
    while True:
        check_alt = bme280.altitude
        print(check_alt)
        GPIO.output(17, GPIO.HIGH)
        sleep(0.25) #might have to increase this to 2
        GPIO.output(17, GPIO.LOW)
        sleep(0.25)
        #check_alt = 9000 #comment this out
        if check_alt-init_alt > 50: #checks if it has gone high, no point taking pictures the way up   
            check_alt = bme280.altitude
            #check_alt=0 #comment this out
            if check_alt-init_alt < 300: #or whatever feet below which we should start taking photos
                #check_alt = 10000 #comment this out
                while check_alt-init_alt>20:
                    GPIO.output(17, GPIO.HIGH)
                    sleep(0.02) #might have to increase this to 2
                    GPIO.output(17, GPIO.LOW)
                    #if the below doesn't work, add r before the first quote
                    current_time = time.time()-start_time
                    camera.capture("/home/pi/Downloads/subscale_test_imgs/img_"+str(img_no)+ "alt: "+str(round(check_alt-init_alt, 4))+"time: "+str(round(current_time,5))+".jpg") #take a picture, for CDR we can probably just save these but we need to get numpy working for analysis
                    img_no+=1
                    altitudes_list.append(["alt"+str(check_alt), "time: "+ str(current_time)])
                    check_alt = bme280.altitude
                    #check_alt = 1000
                break


#so we can look at altitudes corresponding to img#
    with open("altitudes_list", "wb") as fp:
        pickle.dump(altitudes_list,fp)
    with open("altitudes_list", "rb") as fp:
        b=pickle.load(fp)
    print(b)
except:
    print("failed")
    sleep(200)
    for i in range(2000):
        print("picture-taking rn")
        img_no = i
        sleep(2)
        camera.capture("/home/pi/Downloads/subscale_test_imgs/img_%s.jpg" % img_no)
        
        
            