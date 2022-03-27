import logging
import sys
import time
from Adafruit_BNO055 import BNO055
import socket, pickle , time
from math import sqrt
import numpy as np
from scipy import signal

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
timeslist = []
accels = []
sum_time = 0
while sum_time<30:
    current = time.time()
    #sleep(0.07)
    # Read the Euler angles for heading, roll, pitch (all in degrees).
    heading, roll, pitch = bno.read_euler()
    # Read the calibration status, 0=uncalibrated and 3=fully calibrated.
    sys, gyro, accel, mag = bno.get_calibration_status()
   
    final = time.time()-current
    timeslist.append(final)
    accels.append(accel)
    
    sum_time += final
    print('Heading={0:0.2F} Roll={1:0.2F} Pitch={2:0.2F}\tSys_cal={3} Gyro_cal={4} Accel_cal={5} Mag_cal={6}'.format(
          heading, roll, pitch, sys, gyro, accel, mag))
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


    
