import RPi.GPIO as GPIO
from picamera import PiCamera
import board
import digitalio
from adafruit_bme280 import basic as adafruit_bme280
import pickle
from time import sleep

testsave = [1,2,3,4]
with open("testpickle.txt", "wb") as fp:
    pickle.dump(testsave,fp)
with open("testpickle.txt", "rb") as fp:
    b=pickle.load(fp)
print(b)

spi = board.SPI()
bme_cs = digitalio.DigitalInOut(board.D10)
bme280 = adafruit_bme280.Adafruit_BME280_SPI(spi, bme_cs) #sea level pressure at FAR is 29.34Hg which is 993.566hPA
bme280.sea_level_pressure = 1012
altitutes_list = []
img_no = 0
#add some conditional regarding if the button has been pressed to encompass all this

init_alt = bme280.altitude  #bme280 only works with absolute altitude, so we'll establish a baseline on startup at ground
print(init_alt)
while True:
    check_alt = bme280.altitude
    if check_alt-init_alt > -10000: #checks if it has gone high, no point taking pictures the way up   
        print(check_alt-init_alt)
        way_up_or_down+=1
        check_alt = bme280.altitude
        if check_alt-init_alt < 500: #or whatever feet below which we should start taking photos
            while check_alt>0:
                sleep(0.02) #might have to increase this to 2
                #if the below doesn't work, add r before the first quote
                camera.capture("/home/pi/Downloads/testingcam/img_%s.jpg" % img_no) #take a picture, for CDR we can probably just save these but we need to get numpy working for analysis
                img_no+=1
                altitudes_list.append(check_alt)
            break


#so we can look at altitudes corresponding to img#
with open("altitudes_list", "wb") as fp:
    pickle.dump(altitudes_list,fp)
with open("altitudes_list", "rb") as fp:
    b=pickle.load(fp)
print(b)