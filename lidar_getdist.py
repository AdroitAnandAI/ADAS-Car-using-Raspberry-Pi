"""

The getObjectDistance (angle_min, angle_max) function will get the 
distance of LIDAR points between angle_min and angle_max. Then it takes
the median of distances and return it. This function is used as a 
supporting method for the ADAS system after bounding box detection of
objects. The x_min and x_max of bounding boxes are used to compute the
angle_min and angle_max which is passed in to the  getObjectDistance() in 
this module to estimate the object distance.

The code is extensively modified from the base code provided by ADAFruit. 

Adafruit invests time and resources providing this open source code.
Please support Adafruit and open source hardware by purchasing
products from Adafruit!

Written by Dave Astels for Adafruit Industries
Copyright (c) 2019 Adafruit Industries
Licensed under the MIT license.

All text above must be included in any redistribution.
"""

import os
from math import cos, sin, pi, floor
import math
import paho.mqtt.client as mqtt
import pygame
from adafruit_rplidar import RPLidar
import numpy as np

# Screen width & height
W = 640
H = 480

SCAN_BYTE = b'\x20'
SCAN_TYPE = 129


import ledshim
import colorsys
import time
from sys import exit


def make_gaussian(fwhm):
    x = np.arange(0, ledshim.NUM_PIXELS, 1, float)
    y = x[:, np.newaxis]
    x0, y0 = 3.5, (ledshim.NUM_PIXELS / 2) - 0.5
    fwhm = fwhm
    gauss = np.exp(-4 * np.log(2) * ((x - x0) ** 2 + (y - y0) ** 2) / fwhm ** 2)
    return gauss


def flashLight():
    print('inside flashlight')
    for _ in range(10):

        ledshim.set_clear_on_exit()

        for z in list(range(1, 10)[::-1]) + list(range(1, 10)):
            fwhm = 15.0 / z
            gauss = make_gaussian(fwhm)
            print('after make gaussian')
            start = time.time()
            y = 4
            for x in range(ledshim.NUM_PIXELS):
                h = 0.5
                s = 1.0
                v = gauss[x, y]
                rgb = colorsys.hsv_to_rgb(h, s, v)
                r, g, b = [int(255.0 * i) for i in rgb]
                ledshim.set_pixel(x, r, g, b)
            ledshim.show()
            end = time.time()
            t = end - start
            if t < 0.04:
                time.sleep(0.04 - t)


# Setup the RPLidar
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(None, PORT_NAME)

# used to scale data to fit on the screen
max_distance = 0

#pylint: disable=redefined-outer-name,global-statement
def process_data(data):
    global max_distance
    lcd.fill((0,0,0))
    point = ( int(W / 2) , int(H / 2) )
    
    pygame.draw.circle(lcd,pygame.Color(255, 255, 255),point,10 )
    pygame.draw.circle(lcd,pygame.Color(100, 100, 100),point,100 , 1 )
    pygame.draw.line( lcd,pygame.Color(100, 100, 100) , ( 0, int(H/2)),( W , int(H/2) ) )
    pygame.draw.line( lcd,pygame.Color(100, 100, 100) , ( int(W/2),0),( int(W/2) , H ) )

    for angle in range(360):
        distance = data[angle]
        if distance > 0:                  # ignore initially ungathered data points
            max_distance = max([min([5000, distance]), max_distance])
            radians = angle * pi / 180.0
            x = distance * cos(radians)
            y = distance * sin(radians)
            point = ( int(W / 2) + int(x / max_distance * (W/2)), int(H/2) + int(y / max_distance * (H/2) ))
            pygame.draw.circle(lcd,pygame.Color(255, 0, 0),point,2 )
    pygame.display.update()


scan_data = [0]*360

def _process_scan(raw):
    '''Processes input raw data and returns measurment data'''
    new_scan = bool(raw[0] & 0b1)
    inversed_new_scan = bool((raw[0] >> 1) & 0b1)
    quality = raw[0] >> 2
    if new_scan == inversed_new_scan:
        raise RPLidarException('New scan flags mismatch')
    check_bit = raw[1] & 0b1
    if check_bit != 1:
        raise RPLidarException('Check bit not equal to 1')
    angle = ((raw[1] >> 1) + (raw[2] << 7)) / 64.
    distance = (raw[3] + (raw[4] << 8)) / 4.
    return new_scan, quality, angle, distance

def lidar_measurments(self, max_buf_meas=500):
       
        lidar.set_pwm(800)
        status, error_code = self.health
        
        cmd = SCAN_BYTE
        self._send_cmd(cmd)
        dsize, is_single, dtype = self._read_descriptor()
        if dsize != 5:
            raise RPLidarException('Wrong info reply length')
        if is_single:
            raise RPLidarException('Not a multiple response mode')
        if dtype != SCAN_TYPE:
            raise RPLidarException('Wrong response data type')
        while True:
            raw = self._read_response(dsize)
            self.log_bytes('debug', 'Received scan response: ', raw)
            if max_buf_meas:
                data_in_buf = self._serial_port.in_waiting
                if data_in_buf > max_buf_meas*dsize:
                    self.log('warning',
                             'Too many measurments in the input buffer: %d/%d. '
                             'Clearing buffer...' %
                             (data_in_buf//dsize, max_buf_meas))
                    self._serial_port.read(data_in_buf//dsize*dsize)
            yield _process_scan(raw)

def lidar_scans(self, max_buf_meas=500, min_len=5):
        
        scan = []
        iterator = lidar_measurments(lidar,max_buf_meas)
        for new_scan, quality, angle, distance in iterator:
            if new_scan:
                if len(scan) > min_len:
                    yield scan
                scan = []
            if quality > 0 and distance > 0:
                scan.append((quality, angle, distance))

def getObjectDistance (angle_min, angle_max):

    minDist = 0
    lidar = RPLidar(None, PORT_NAME)

    try:
            for scan in lidar_scans(lidar):

                for (_, angle, distance) in scan:
                    # print("Angle = " + str(angle) + "distance == " + str(distance))
                    scan_data[min([359, floor(angle)])] = distance


                # fetching all non zero distance values between subtended angles
                allDists = [scan_data[i] for i in range(360)
                                if i >= angle_min and i <= angle_max and scan_data[i] > 0]

                # if half the distance values are filled in then break
                if (2 * len(allDists) > angle_max - angle_min):

                    minDist = np.median(allDists)
                    lidar.stop()
                    lidar.disconnect()

                    return minDist

    except KeyboardInterrupt:
        print('Stopping LIDAR Scan')

def roundtoTen(x):
    return int(math.ceil(x/10.0)) * 10

def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("object/getdistance")


def on_message(client, userdata, msg):

    # print(msg.payload.decode())

    word = msg.payload.decode()

    # objAttributes contains label,
    # theta min and max separated by |
    objAttributes = word.split('|')

    now = time.localtime()
    if (now.tm_min * 60 + now.tm_sec - int(objAttributes[3])  >= 1):
        return

    theta1 = float(objAttributes[1])
    theta2 = float(objAttributes[2])

    dist = getObjectDistance(int(theta1) + 90 + 59, int(theta2) + 90 + 59)

    # convert distance from mm to cms
    dist = round(float (dist / 1000), 1)

    # print('The distance to the object = ' + str(dist))

    theta_mid = int((theta1 + theta2) / 2)
    print(' Object is at an angle of ' + str(theta_mid))

    # if near then announce an alert!
    # Passing the hue value on MQTT. 0 = Red. 0.3 = Green
    if (dist < 2.0):
        # print('setting alert msg')
        announceText = "ALERT ALERT "
        client.publish("object/flashlight", "0.0")
    else:
        announceText = ""
        client.publish("object/flashlight", "0.3")

    announceText = announceText + str(objAttributes[0]) + ' at ' + str(dist) + ' meters. '

    # theta_mid can vary from 0 to 62 degrees
    if theta_mid > 40:
        # print('Right Side')
        os.system(
            'espeak \"' + announceText + str (roundtoTen(abs(theta_mid - 31))) + ' degrees right\"')
    elif theta_mid < 21:
        # print('Left Side')
        os.system(
            'espeak \"' + announceText + str (roundtoTen(abs(31 - theta_mid))) + ' degrees left\"')
    else:
        # theta_mid will be > 20 and < 40, if here
        if theta_mid < 30:
            direction = ' Right '
        else:
            direction = ' Left '
        # Alert to slide to opposite direction at theta + 30 degrees
        os.system(
            'espeak \"' + announceText + 'Slide ' + direction + str(abs(roundtoTen(abs(31 - theta_mid)) + 30)) + 'degrees \"')


client = mqtt.Client()
client.connect("localhost", 1883, 600)

client.on_connect = on_connect
client.on_message = on_message

try:
    client.loop_forever()
# To catch SigINT
except KeyboardInterrupt:
    client.disconnect()
