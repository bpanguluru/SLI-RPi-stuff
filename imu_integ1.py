import logging
import sys
import time
#from Adafruit_BNO055 import BNO055
import socket, pickle , time
from math import sqrt
import numpy as np
from scipy import signal
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

test_acc = sensor.acceleration
test_gyro = sensor.gyro
test_q = sensor.quaternion
test_euler = sensor.euler
test_linear_acc = sensor.linear_acceleration
grav = sensor.gravity

timeslist = []
accels = []
q_list = []
sum_time = 0
while sum_time<30:
    current = time.time()
    #sleep(0.07)   
    final = time.time()-current
    timeslist.append(final)
    accels.append(sensor.acceleration)
    quat = sensor.quaternion
    q_list.append(quat)
    sum_time += final
    
#put the read, save to file, and append to list into a try: except: pass in the main code
#also add a try: except: pass to the receiver code to prevent byte error    

def read_imu(data_package1):
    euler_data = [data_package1[0],data_package1[1], data_package1[2]]
    accel_data = [data_package1[3]/9.81,data_package1[4]/9.81, data_package1[5]/9.81]
    linear_data = [data_package1[6]/9.81, data_package1[7]/9.81, data_package1[8]/9.81]
    q = [data_package1[9], data_package1[10], data_package1[11], data_package1[12]]
    # q = [ data package1[10], data package1[11], data package1[12],data package1[9]]
    return euler_data, accel_data, linear_data , q
def quaternconj(q):
    q = [q[0], −q[1], −q[2], −q[3]]
    return q
def quaternprod(a ,b):
    ab = [0, 0, 0, 0]
    ab[0] = a[0] ∗ b[0] − a[1] ∗ b[1] − a[2] ∗ b[2] − a[3] ∗ b[3]
    ab[1] = a[0] ∗ b[1] + a[1] ∗ b[0] + a[2] ∗ b[3] − a[3] ∗ b[2]
    59
    ab[2] = a[0] ∗ b[2] − a[1] ∗ b[3] + a[2] ∗ b[0] + a[3] ∗ b[1]
    ab[3] = a[0] ∗ b[3] + a[1] ∗ b[2] − a[2] ∗ b[1] + a[3] ∗ b[0]
    return ab
def quaternrotate(acc, q):
    x = quaternprod(q, [0, acc[0], acc[1], acc[2]])
    y = quaternprod(x , quaternconj(q))
    z = np.array([y[1], y[2], y[3]])
    return z
def reset_data ():
    lin_accel_old = np.zeros((1, 3))
    linVel_old= np.zeros((1, 3))
    linPos_old = np.array((-2.4, 0, 0))
    return linVel_old , lin_accel_old , linPos_old

lin_accel_old = np.zeros((1,3))
linVel_old = np.zeros((1,3))
linVel_new = np.zeros((1,3))
linPos_old = np.zeros((1,3))
linPos_new = np.zeros((1,3))
steptime = 0.12
P_x = []
P_y = []
samplePeriod = 1/100
ACCELEROMETER_DRIFT_WHEN_STATIONARY = 2.3*e−26
countaccX = 0
countaccY = 0

accMagnitude = sqrt((lin_acc[0] ∗ lin_acc[0]) + (lin_acc[1] ∗ lin_acc[1]) + (lin_acc[2] ∗ lin_acc[2]) )
# print(accMagnitude)
filterCutoff = 0.001
butterFilterB, butterFilterA = signal.butter(1, (2 ∗ filterCutoff ) / (1/samplePeriod), "highpass")
accMagnitudeFiltered = signal.filtfilt(butterFilterB, butterFilterA, [accMagnitude,accMagnitude], padlen=1)
accMagnitudeFiltered = abs(accMagnitudeFiltered)
# print(accMagnitudeFiltered)

filterCutoff = 5
butterFilterB, butterFilterA = signal.butter(1, (2 ∗ filterCutoff ) / (1/samplePeriod), "lowpass")
accMagnitudeFiltered = signal.filtfilt (butterFilterB, butterFilterA,[accMagnitudeFiltered, accMagnitudeFiltered], padlen=1)
stationary = accMagnitudeFiltered < ACCELEROMETER_DRIFT_WHEN_STATIONARY

if stationary.any():
linVel_new =np.zeros((1, 3))
linPos_new += linVel_new ∗ steptime
P_x.append(linPos_new[0, 0])
P_y.append(linPos_new[0, 1])
lin_accel_old = np.zeros((1, 3))
linVel_old = np.zeros((1,3))
conetinue

Acc = [lin_acc [0] ∗ 9.81, lin_acc [1] ∗ 9.81, lin_acc [2] ∗ 9.81]
acc_new = np.matrix(quaternrotate(Acc, quaternconj(q)))

leakRateAcc = 1
linVel_new = linVel_new ∗ leakRateAcc + ((lin_accel_old + ((acc_new − lin_accel_old)/2)) ∗ steptime)
lin_accel_old = acc_new

leakRatevel = 1
linPos_new = linPos_new ∗ leakRatevel + (linVel_old + ((linVel_new − linVel_old)/2)) ∗ steptime
linVel_old = linVel_new




    
