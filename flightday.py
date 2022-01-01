import RPi.GPIO as GPIO
from picamera import PiCamera
import board
import digitalio
from adafruit_bme280 import basic as adafruit_bme280
import pickle
import time
from time import sleep

camera=PiCamera()
start_time = time.time()

try:
    i2c = board.I2C()
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
    #bme_cs = digitalio.DigitalInOut(board.D10)
    #bme280 = adafruit_bme280.Adafruit_BME280_SPI(spi, bme_cs) #sea level pressure at FAR is 29.34Hg which is 993.566hPA
    bme280.sea_level_pressure = 993.566 # don't need to do this
    altitutes_list = []
    img_no = 0
    #add some conditional regarding if the button has been pressed to encompass all this

    init_alt = 0

    for i in range(20):
        calib_alt = bme280.altitude  #bme280 only works with absolute altitude, so we'll establish a baseline on startup at ground
        init_alt+=calib_alt
        init_alt = init_alt/20

    bme280.sea_level_pressure = init_alt
    passedUpPoint = False

    while (passedUpPoint==True or bme280.altitude > 50): # this may malfunction due to black powder charge
        sleep(0.02) #might have to increase this to 2
        check_alt = bme280.altitude
        if check_alt > 50: #take pictures whenever alt is greater than 50 meters
            #if the below doesn't work, add r before the first quote
            current_time = time.time()-start_time
            camera.capture("/home/pi/Downloads/subscale_test_imgs/img_" + str(img_no) + "_height" + str(bme280.altitude) + "m_time" + str(current_time)+".jpg") #take a picture, for CDR we can probably just save these but we need to get numpy working for analysis
            altitudes_list.append(["alt: "+ str(check_alt), "time: "+ str(current_time), "img_no: " + str(img_no)])
            img_no+=1
        if check_alt > 100:
            passedUpPoint = True


#so we can look at altitudes corresponding to img#
    with open("altitudes_list", "wb") as fp:
        pickle.dump(altitudes_list,fp)
    with open("altitudes_list", "rb") as fp:
        b=pickle.load(fp)
    print(b)
except:
    print("failed")
    sleep(50)
    for i in range(5000):
        print("picture-taking rn")
        img_no = i
        sleep(0.5)
        camera.capture("/home/pi/Downloads/subscale_test_imgs/img_%s.jpg" % img_no)
