#!/usr/bin/env python

import sys
from serial import *
import time
import csv


def main():
    arduino = Serial(port='/dev/cu.usbserial-A602ZFJV', baudrate=9600)
    outputFile = open(sys.argv[1], 'wt')
    logging = True;
    timeStamp = time.time();
    time.sleep(5)
    
    while logging:
        gyro = arduino.readline()
        print gyro
        outputFile.write(gyro)
        if time.time() - timeStamp > 20:
            logging = False
            
    outputFile.close()
    arduino.close()
    
main()
