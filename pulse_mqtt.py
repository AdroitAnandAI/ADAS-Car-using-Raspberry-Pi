#!/usr/bin/env python

import os
from math import cos, sin, pi, floor
import paho.mqtt.client as mqtt
import colorsys
import time
from sys import exit

try:
    import numpy as np
except ImportError:
    exit('This script requires the numpy module\nInstall with: sudo pip install numpy')

import ledshim

ledshim.set_clear_on_exit()


def make_gaussian(fwhm):
    x = np.arange(0, ledshim.NUM_PIXELS, 1, float)
    y = x[:, np.newaxis]
    x0, y0 = 3.5, (ledshim.NUM_PIXELS / 2) - 0.5
    fwhm = fwhm
    gauss = np.exp(-4 * np.log(2) * ((x - x0) ** 2 + (y - y0) ** 2) / fwhm ** 2)
    return gauss


def flashLight(colorCode):
    # print('inside flashlight')
    for _ in range(5):

        ledshim.set_clear_on_exit()

        for z in list(range(1, 10)[::-1]) + list(range(1, 10)):
            fwhm = 15.0 / z
            gauss = make_gaussian(fwhm)

            start = time.time()
            y = 4
            for x in range(ledshim.NUM_PIXELS):
                h = colorCode
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



def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("object/flashlight")


def on_message(client, userdata, msg):

    colorCode = msg.payload.decode()
    flashLight(float(colorCode))


client = mqtt.Client()
client.connect("localhost", 1883, 600)

client.on_connect = on_connect
client.on_message = on_message

try:
    client.loop_forever()
# To catch SigINT
except KeyboardInterrupt:
    client.disconnect()