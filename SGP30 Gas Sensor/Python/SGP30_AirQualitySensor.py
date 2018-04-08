##*****************************************************
## SGP30 Air Quality Sensor Script
## Author:          MikrocontrollerProjekte
## Contact:         https://www.youtube.com/c/MikrocontrollerProjekte
##                  https://github.com/MikrocontollerProjekte/ 
## Date:            17.03.2018
SGP30_SCRIPT_VERSION = 0.1
##*****************************************************


## Imports
import serial
import time
from datetime import datetime
import fileinput
import os
import sys


## configure the serial connections (the parameters differs on the device you are connecting to)
ser = serial.Serial(
    port='COM5', ## <--- change to your COM Port here
	baudrate=115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout= None
)

try:
    ser.open()  ## open port
except serial.SerialException:
    ser.close() ## close if it is open
    ser.open()  ## open port


## Main Program
print("\n")
print("***************************************")
print("*** SGP30 Air Quality Sensor Script ***")
print("***************************************")
print("\n")
print("\nRS232 is connected to: " + ser.portstr + "   Baudrate: " + str(ser.baudrate)+ "   Parity: " + str(ser.parity)+ "   StopBits: " + str(ser.stopbits)+ "   DataBits: " + str(ser.bytesize))
print("\n")


## create a Sensor Logfile
try:
    FilenameAndPath = (os.path.abspath(sys.argv[0]))
    head, tail = os.path.split(FilenameAndPath)
    sensorLogFile = open(head + "/" + datetime.utcnow().strftime('%Y_%m_%d %H_%M_%S') + ".csv", "wt", 1) ## create log file
except IOError:
    print("\nCould not create/modify Logfile! \n")


sensorLogFile.writelines("time [hr:min:sec]; tVOC Concentration [ppb]; CO2eq Concentration [ppm]; Measurement No.\n") ## write first a data description line


## start log
while 1:
    try:
        line = ser.readline()
        ## write every incoming Line to Logfile and console with Timestamp
        sensorData = datetime.utcnow().strftime('%H:%M:%S') + " Received Line: " + str(line)
        print(sensorData)

        if ((sensorData.find("tVOC Concentration: ") != -1) and (sensorData.find("CO2eq Concentration: ") != -1) and (sensorData.find("measurement no.: ") != -1)):
            value_TVOC = sensorData[sensorData.find("tVOC Concentration: ") + 20 : sensorData.find(" [ppb]")]
            value_eCO2 = sensorData[sensorData.find("CO2eq Concentration: ") + 21 : sensorData.find(" [ppm]")]
            measurementNo = sensorData[sensorData.find("measurement no.: ") + 17 : sensorData.find("measurement no.: ") + 27]
            sensorLogFile.writelines(datetime.utcnow().strftime('%H:%M:%S') + ";" + value_TVOC.strip() + ";" + value_eCO2.strip() + ";" + measurementNo.strip() + "\n")
    except serial.SerialException:
        print('Exit now\n')
        ser.close()
        sensorLogFile.close() 
        time.sleep(9)
        break