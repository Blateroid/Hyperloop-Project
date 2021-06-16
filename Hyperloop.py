#Libraries

#Pressure Libraries
import time
import smbus
import math

#IMU Libraries
import sys
import IMU
import datetime
import os

#Distance Libraries
import RPi.GPIO as GPIO

#ESC Liraries
import signal
import atexit
import random

#GUI
from Tkinter import Scale, Tk, Frame, Label, Button
from ttk import Notebook,Entry

#******************** END LIBRARIES********************

#Variables

#Pressure
# define BMP388 Device I2C address
I2C_ADD_BMP388_AD0_LOW    =    0x76
I2C_ADD_BMP388_AD0_HIGH    =   0x77
I2C_ADD_BMP388    =        I2C_ADD_BMP388_AD0_LOW

BMP388_REG_ADD_WIA    =    0x00
BMP388_REG_VAL_WIA        =    0x50

BMP388_REG_ADD_ERR    =    0x02
BMP388_REG_VAL_FATAL_ERR    =    0x01
BMP388_REG_VAL_CMD_ERR        =    0x02
BMP388_REG_VAL_CONF_ERR        =    0x04

BMP388_REG_ADD_STATUS    =    0x03
BMP388_REG_VAL_CMD_RDY        =    0x10
BMP388_REG_VAL_DRDY_PRESS    =    0x20
BMP388_REG_VAL_DRDY_TEMP    =    0x40

BMP388_REG_ADD_CMD    =    0x7E
BMP388_REG_VAL_EXTMODE_EN    =    0x34
BMP388_REG_VAL_FIFI_FLUSH    =    0xB0
BMP388_REG_VAL_SOFT_RESET    =    0xB6

BMP388_REG_ADD_PWR_CTRL    =    0x1B
BMP388_REG_VAL_PRESS_EN        =    0x01
BMP388_REG_VAL_TEMP_EN        =    0x02
BMP388_REG_VAL_NORMAL_MODE    =    0x30

BMP388_REG_ADD_PRESS_XLSB    =    0x04
BMP388_REG_ADD_PRESS_LSB    =    0x05
BMP388_REG_ADD_PRESS_MSB    =    0x06
BMP388_REG_ADD_TEMP_XLSB    =    0x07
BMP388_REG_ADD_TEMP_LSB        =    0x08
BMP388_REG_ADD_TEMP_MSB        =    0x09

BMP388_REG_ADD_T1_LSB    =    0x31
BMP388_REG_ADD_T1_MSB    =    0x32
BMP388_REG_ADD_T2_LSB    =    0x33
BMP388_REG_ADD_T2_MSB    =    0x34
BMP388_REG_ADD_T3        =    0x35
BMP388_REG_ADD_P1_LSB    =    0x36
BMP388_REG_ADD_P1_MSB    =    0x37
BMP388_REG_ADD_P2_LSB    =    0x38
BMP388_REG_ADD_P2_MSB    =    0x39
BMP388_REG_ADD_P3        =    0x3A
BMP388_REG_ADD_P4        =    0x3B
BMP388_REG_ADD_P5_LSB    =    0x3C
BMP388_REG_ADD_P5_MSB    =    0x3D
BMP388_REG_ADD_P6_LSB    =    0x3E
BMP388_REG_ADD_P6_MSB    =    0x3F
BMP388_REG_ADD_P7        =    0x40
BMP388_REG_ADD_P8        =    0x41
BMP388_REG_ADD_P9_LSB    =    0x42
BMP388_REG_ADD_P9_MSB    =    0x43
BMP388_REG_ADD_P10        =    0x44
BMP388_REG_ADD_P11        =    0x45

#IMU
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070          # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40              # Complementary filter constant
MAG_LPF_FACTOR = 0.4    # Low pass filter constant magnetometer
ACC_LPF_FACTOR = 0.4    # Low pass filter constant for accelerometer
ACC_MEDIANTABLESIZE = 9         # Median filter table size for accelerometer. Higher = smoother but a longer delay
MAG_MEDIANTABLESIZE = 9         # Median filter table size for magnetometer. Higher = smoother but a longer delay

################# Compass Calibration values ############
# Use calibrateBerryIMU.py to get calibration values
# Calibrating the compass isnt mandatory, however a calibrated
# compass will result in a more accurate heading value.

magXmin =  -2743
magYmin =  -1896
magZmin =  -522
magXmax =  234
magYmax =  738
magZmax =  1279

############### END Calibration offsets #################

gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
oldXMagRawValue = 0
oldYMagRawValue = 0
oldZMagRawValue = 0
oldXAccRawValue = 0
oldYAccRawValue = 0
oldZAccRawValue = 0

#distance - no variables

#Algorithm
u = 0
rateofchange = 0.02
DisplacementX = 0.00
DisplacementY = 0.00
DisplacementZ = 0.00
r = 0
t = 0
j = 0
#********************* END VARIABLES****************************

#Classes and methods

#Pressure
class BMP388(object):
    """docstring for BMP388"""
    def __init__(self, address=I2C_ADD_BMP388):
        self._address = address
        self._bus = smbus.SMBus(1)
        # Load calibration values.
        if self._read_byte(BMP388_REG_ADD_WIA) == BMP388_REG_VAL_WIA:
            print("Pressure sersor is BMP388!\r\n")    
            u8RegData = self._read_byte(BMP388_REG_ADD_STATUS)
            if ( u8RegData & BMP388_REG_VAL_CMD_RDY ):
                self._write_byte(BMP388_REG_ADD_CMD, BMP388_REG_VAL_SOFT_RESET) 
                time.sleep(0.01)
        else:
            print("Pressure sersor NULL!\r\n")
        self._write_byte( BMP388_REG_ADD_PWR_CTRL,BMP388_REG_VAL_PRESS_EN | BMP388_REG_VAL_TEMP_EN | BMP388_REG_VAL_NORMAL_MODE)
        self._load_calibration()

    def _read_byte(self,cmd):
        return self._bus.read_byte_data(self._address,cmd)

    def _read_s8(self,cmd):
        result = self._read_byte(cmd)
        if result > 128:result -= 256
        return result    

    def _read_u16(self,cmd):
        LSB = self._bus.read_byte_data(self._address,cmd)
        MSB = self._bus.read_byte_data(self._address,cmd+1)
        return (MSB    << 8) + LSB

    def _read_s16(self,cmd):
        result = self._read_u16(cmd)
        if result > 32767:result -= 65536
        return result

    def _write_byte(self,cmd,val):
        self._bus.write_byte_data(self._address,cmd,val)

    def _load_calibration(self):
        print("_load_calibration\r\n")
        "load calibration"
        """ read the temperature calibration parameters """
        self.T1 =self._read_u16(BMP388_REG_ADD_T1_LSB)
        self.T2 =self._read_u16(BMP388_REG_ADD_T2_LSB)
        self.T3 =self._read_s8(BMP388_REG_ADD_T3)
        """ read the pressure calibration parameters """
        self.P1 =self._read_s16(BMP388_REG_ADD_P1_LSB)
        self.P2 =self._read_s16(BMP388_REG_ADD_P2_LSB)
        self.P3 =self._read_s8(BMP388_REG_ADD_P3)
        self.P4 =self._read_s8(BMP388_REG_ADD_P4)
        self.P5 =self._read_u16(BMP388_REG_ADD_P5_LSB)
        self.P6 =self._read_u16(BMP388_REG_ADD_P6_LSB)
        self.P7 =self._read_s8(BMP388_REG_ADD_P7)
        self.P8 =self._read_s8(BMP388_REG_ADD_P8)
        self.P9 =self._read_s16(BMP388_REG_ADD_P9_LSB)    
        self.P10 =self._read_s8(BMP388_REG_ADD_P10)
        self.P11=self._read_s8(BMP388_REG_ADD_P11)
        #print(self.T1)
        #print(self.T2)
        #print(self.T3)
        #print(self.P1)
        #print(self.P2)
        #print(self.P3)
        #print(self.P4)
        #print(self.P5)
        #print(self.P6)
        #print(self.P7)
        #print(self.P8)
        #print(self.P9)
        #print(self.P10)
        #print(self.P11)
    def compensate_temperature(self,adc_T):
        partial_data1 =  (adc_T - (256 *  (self.T1)))
        partial_data2 =  (self.T2 * partial_data1)
        partial_data3 =  (partial_data1 * partial_data1)
        partial_data4 =  (( partial_data3) * ( self.T3))
        partial_data5 = ( (( partial_data2) * 262144) +  partial_data4)
        partial_data6 =  (( partial_data5) / 4294967296)
        self.T_fine = partial_data6
        comp_temp =  ((partial_data6 * 25)  / 16384)
        return comp_temp;   
            
    def compensate_pressure(self,adc_P):
        partial_data1 = self.T_fine * self.T_fine
        partial_data2 = partial_data1 / 64
        partial_data3 = (partial_data2 * self.T_fine) / 256
        partial_data4 = (self.P8 * partial_data3) / 32
        partial_data5 = (self.P7 * partial_data1) * 16
        partial_data6 = (self.P6 * self.T_fine) * 4194304;
        offset =  ( (self.P5) *  140737488355328) + partial_data4 + partial_data5 + partial_data6

        partial_data2 = (( self.P4) * partial_data3) / 32
        partial_data4 = (self.P3 * partial_data1) * 4
        partial_data5 = ( (self.P2) - 16384) * ( self.T_fine) * 2097152
        sensitivity = (( (self.P1) - 16384) *  70368744177664) + partial_data2 + partial_data4 + partial_data5

        partial_data1 = (sensitivity / 16777216) * adc_P
        partial_data2 =  (self.P10) *  (self.T_fine)
        partial_data3 = partial_data2 + (65536 *  (self.P9))
        partial_data4 = (partial_data3 * adc_P) / 8192
        partial_data5 = (partial_data4 * adc_P) / 512
        partial_data6 =  ( adc_P *  adc_P)
        partial_data2 = ( (self.P11) *  (partial_data6)) / 65536
        partial_data3 = (partial_data2 * adc_P) / 128
        partial_data4 = (offset / 4) + partial_data1 + partial_data5 + partial_data3
        comp_press = (( partial_data4 * 25) /  1099511627776)
        return comp_press;

    def get_temperature_and_pressure_and_altitude(self):
        """Returns pressure in Pa as double. Output value of "6386.2"equals 96386.2 Pa = 963.862 hPa."""
        xlsb = self._read_byte(BMP388_REG_ADD_TEMP_XLSB)
        lsb =  self._read_byte(BMP388_REG_ADD_TEMP_LSB)
        msb =  self._read_byte(BMP388_REG_ADD_TEMP_MSB)
        adc_T = (msb << 16) + (lsb << 8) + (xlsb)
        temperature = self.compensate_temperature(adc_T)
        xlsb = self._read_byte(BMP388_REG_ADD_PRESS_XLSB)
        lsb =  self._read_byte(BMP388_REG_ADD_PRESS_LSB) 
        msb =  self._read_byte(BMP388_REG_ADD_PRESS_MSB) 

        adc_P = (msb << 16) + (lsb << 8) + (xlsb)
        pressure = self.compensate_pressure(adc_P)
        altitude  = 4433000 * (1 - pow(((pressure/100.0) / 101325.0), 0.1903)) 

        return temperature,pressure,altitude
def pressureMethod():
    try:
        #time.sleep(0.5)
        temperature,pressure,altitude = bmp388.get_temperature_and_pressure_and_altitude()
        finalTemperature = temperature/100.0
        finalPressure = pressure/100.0
        finalAltitude = altitude/100.0
        #print(' Temperature = %.1f Pressure = %.2f  Altitude =%.2f '%(finalTemperature,finalPressure,finalAltitude))
        return finalTemperature, finalPressure, finalAltitude
    except IOError as e:
        print("IO error detected")
        print(e)

    
#IMU - no methods
    
#Distance

def checkdist():
	GPIO.output(16, GPIO.HIGH)
	time.sleep(0.000015)
	GPIO.output(16, GPIO.LOW)
	while not GPIO.input(18):
		pass
	t1 = time.time()
	while GPIO.input(18):
		pass
	t2 = time.time()
	return(t2-t1)*340/2

def distanceMethod():
    distancevalue = checkdist()
    #print 'Distance: %0.3f m' %distancevalue
    time.sleep(0.5)
       
    return distancevalue

#ESC

#GUI
def displaytable_constants():
  label = Label(window, text="Source", bg="black", fg="white", padx=3, pady=3)
  label.config(font=('Arial', 14))
  label.grid(row=0, column=0, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(0, weight=1)

  label = Label(window, text="Value x", bg="black", fg="white", padx=3, pady=3)
  label.config(font=('Arial', 14))
  label.grid(row=0, column=1, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(1, weight=1)

  label = Label(window, text="Value y", bg="black", fg="white", padx=3, pady=3)
  label.config(font=('Arial', 14))
  label.grid(row=0, column=2, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(2, weight=1)

  label = Label(window, text="Value z", bg="black", fg="white", padx=3, pady=3)
  label.config(font=('Arial', 14))
  label.grid(row=0, column=3, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(3, weight=1)

  label = Label(window, text="Temperature", bg="light grey",fg="black",padx=3,pady=3)
  label.grid(row=4, column=0, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(0, weight=1)
  
  label = Label(window, text="Pressure", bg="light grey",fg="black",padx=3,pady=3)
  label.grid(row=4, column=1, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(1, weight=1)
  
  label = Label(window, text="Acceleration", bg="light grey",fg="black",padx=3,pady=3)
  label.grid(row=1, column=0, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(0, weight=1)
  
  label = Label(window, text="Displacement", bg="light grey",fg="black",padx=3,pady=3)
  label.grid(row=3, column=0, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(0, weight=1)
  
  label = Label(window, text="Distance", bg="light grey",fg="black",padx=3,pady=3)
  label.grid(row=4, column=2, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(2, weight=1)
  
  label = Label(window, text="Angles", bg="light grey",fg="black",padx=3,pady=3)
  label.grid(row=2, column=0, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(0, weight=1)

  label = Label(window, text="RPM %", bg="light grey",fg="black",padx=3,pady=3)
  label.grid(row=4, column=3, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(3, weight=1)
#  window.after(1000,displaytable)

def displaytable_content(temp,press,accx,accy,accz,dx,dy,dz,dis,angx,angy,angz,rpm):
  label = Label(window, text="%3.1f celcius" % temp, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=5, column=0, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(0, weight=1)
  
  label = Label(window, text="%3.1f Pascals" % press, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=5, column=1, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(1, weight=1)
  
  label = Label(window, text="%3.1f m/s^2" % accx, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=1, column=1, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(1, weight=1)
  
  label = Label(window, text="%3.1f m/s^2" % accy, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=1, column=2, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(2, weight=1)
  
  label = Label(window, text="%3.1f m/s^2" % accz, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=1, column=3, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(3, weight=1)
  
  label = Label(window, text="%3.1f meters" % dx, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=3, column=1, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(1, weight=1)

  label = Label(window, text="%3.1f meters" % dy, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=3, column=2, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(2, weight=1)

  label = Label(window, text="%3.1f meters" % dz, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=3, column=3, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(3, weight=1)
  
  label = Label(window, text="%3.1f cm" % dis, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=5, column=2, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(2, weight=1)
  
  label = Label(window, text="%3.1f degrees" % angx, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=2, column=1, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(1, weight=1)
  
  label = Label(window, text="%3.1f degrees" % angy, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=2, column=2, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(2, weight=1)
  
  label = Label(window, text="%3.1f degrees" % angz, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=2, column=3, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(3, weight=1)
  
  label = Label(window, text="%3.1f percent" % rpm, bg="white",fg="black",padx=3,pady=3)
  label.grid(row=5, column=3, sticky="nsew", padx=1, pady=1)
  window.grid_columnconfigure(3, weight=1)

#***************** END METHODS/Classes***************************

#Code before loop

#Pressure
bmp388 = BMP388()
#IMU
a = datetime.datetime.now()

#Setup the tables for the mdeian filter. Fill them all with '1' so we dont get devide by zero error
acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE

IMU.detectIMU()     #Detect if BerryIMU is connected.
if(IMU.BerryIMUversion == 99):
    print(" No BerryIMU found... exiting ")
    sys.exit()
IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

#Distance
GPIO.setmode(GPIO.BOARD)
GPIO.setup(16,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(18,GPIO.IN)
time.sleep(2)

#ESCs
atexit.register(GPIO.cleanup)

servopin = 11
maximum = 10
minimum = 5
print("starting program")
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servopin,GPIO.OUT, initial=False)
p = GPIO.PWM(servopin, 55)
p.start(minimum)
time.sleep(5)
speed = 10.0

#GUI
window=Tk()
window.title("Hyperloop")
displaytable_constants()

#************** END OF CODE BEFORE LOOP****************************************
      
#While Loop CODE
while True:

    
    #Pressure
    temperature,pressure,altitude = pressureMethod()                  #VARIABLES TO PRINT IN GUI
    #print (temperature, pressure, altitude)
    #IMU
    #Read the accelerometer,gyroscope and magnetometer values
    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()
    GYRx = IMU.readGYRx()
    GYRy = IMU.readGYRy()
    GYRz = IMU.readGYRz()
    MAGx = IMU.readMAGx()
    MAGy = IMU.readMAGy()
    MAGz = IMU.readMAGz()


    #Apply compass calibration
    MAGx -= (magXmin + magXmax) /2
    MAGy -= (magYmin + magYmax) /2
    MAGz -= (magZmin + magZmax) /2


    ##Calculate loop Period(LP). How long between Gyro Reads
    b = datetime.datetime.now() - a
    a = datetime.datetime.now()
    LP = b.microseconds/(1000000*1.0)
    outputString = "Loop Time %5.2f " % ( LP )



    ###############################################
    #### Apply low pass filter ####
    ###############################################
    oldXMagRawValue = MAGx
    oldYMagRawValue = MAGy
    oldZMagRawValue = MAGz
    oldXAccRawValue = ACCx
    oldYAccRawValue = ACCy
    oldZAccRawValue = ACCz
    
    MAGx =  MAGx  * MAG_LPF_FACTOR + oldXMagRawValue*(1 - MAG_LPF_FACTOR);
    MAGy =  MAGy  * MAG_LPF_FACTOR + oldYMagRawValue*(1 - MAG_LPF_FACTOR);
    MAGz =  MAGz  * MAG_LPF_FACTOR + oldZMagRawValue*(1 - MAG_LPF_FACTOR);
    ACCx =  ACCx  * ACC_LPF_FACTOR + oldXAccRawValue*(1 - ACC_LPF_FACTOR);
    ACCy =  ACCy  * ACC_LPF_FACTOR + oldYAccRawValue*(1 - ACC_LPF_FACTOR);
    ACCz =  ACCz  * ACC_LPF_FACTOR + oldZAccRawValue*(1 - ACC_LPF_FACTOR);

    #########################################
    #### Median filter for accelerometer ####
    #########################################
    # cycle the table
    for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ):
        acc_medianTable1X[x] = acc_medianTable1X[x-1]
        acc_medianTable1Y[x] = acc_medianTable1Y[x-1]
        acc_medianTable1Z[x] = acc_medianTable1Z[x-1]

    # Insert the lates values
    acc_medianTable1X[0] = ACCx
    acc_medianTable1Y[0] = ACCy
    acc_medianTable1Z[0] = ACCz

    # Copy the tables
    acc_medianTable2X = acc_medianTable1X[:]
    acc_medianTable2Y = acc_medianTable1Y[:]
    acc_medianTable2Z = acc_medianTable1Z[:]

    # Sort table 2
    acc_medianTable2X.sort()
    acc_medianTable2Y.sort()
    acc_medianTable2Z.sort()

    # The middle value is the value we are interested in
    ACCx = acc_medianTable2X[int(ACC_MEDIANTABLESIZE/2)];
    ACCy = acc_medianTable2Y[int(ACC_MEDIANTABLESIZE/2)];
    ACCz = acc_medianTable2Z[int(ACC_MEDIANTABLESIZE/2)];



    #########################################
    #### Median filter for magnetometer ####
    #########################################
    # cycle the table
    for x in range (MAG_MEDIANTABLESIZE-1,0,-1 ):
        mag_medianTable1X[x] = mag_medianTable1X[x-1]
        mag_medianTable1Y[x] = mag_medianTable1Y[x-1]
        mag_medianTable1Z[x] = mag_medianTable1Z[x-1]

    # Insert the latest values
    mag_medianTable1X[0] = MAGx
    mag_medianTable1Y[0] = MAGy
    mag_medianTable1Z[0] = MAGz

    # Copy the tables
    mag_medianTable2X = mag_medianTable1X[:]
    mag_medianTable2Y = mag_medianTable1Y[:]
    mag_medianTable2Z = mag_medianTable1Z[:]

    # Sort table 2
    mag_medianTable2X.sort()
    mag_medianTable2Y.sort()
    mag_medianTable2Z.sort()

    # The middle value is the value we are interested in
    MAGx = mag_medianTable2X[int(MAG_MEDIANTABLESIZE/2)];
    MAGy = mag_medianTable2Y[int(MAG_MEDIANTABLESIZE/2)];
    MAGz = mag_medianTable2Z[int(MAG_MEDIANTABLESIZE/2)];


    #Convert Gyro raw to degrees per second
    rate_gyr_x =  GYRx * G_GAIN
    rate_gyr_y =  GYRy * G_GAIN
    rate_gyr_z =  GYRz * G_GAIN


    #Calculate the angles from the gyro.
    gyroXangle+=rate_gyr_x*LP
    gyroYangle+=rate_gyr_y*LP                  #VARIABLES TO PRINT IN GUI
    gyroZangle+=rate_gyr_z*LP
    

    #Convert Accelerometer values to degrees
    AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
    AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG


    #Change the rotation value of the accelerometer to -/+ 180 and
    #move the Y axis '0' point to up.  This makes it easier to read.
    if AccYangle > 90:
        AccYangle -= 270.0
    else:
        AccYangle += 90.0

    #Calculate heading
    heading = 180 * math.atan2(MAGy,MAGx)/M_PI

    #Only have our heading between 0 and 360
    if heading < 0:
        heading += 360

    ####################################################################
    ###################Tilt compensated heading#########################
    ####################################################################
    #Normalize accelerometer raw values.
    accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)


    #Calculate pitch and roll
    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm/math.cos(pitch))


    #Calculate the new tilt compensated values
    #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
    #This needs to be taken into consideration when performing the calculations

    #X compensation
    if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
        magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
    else:                                                                #LSM9DS1
        magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)

    #Y compensation
    if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
    else:                                                                #LSM9DS1
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)





    #Calculate tilt compensated heading
    tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

    if tiltCompensatedHeading < 0:
        tiltCompensatedHeading += 360


    ##################### END Tilt Compensation ########################
        
    #Read the accelerometer,gyroscope and magnetometer values
    ACCx2 = IMU.readACCx()
    ACCy2 = IMU.readACCy()
    ACCz2 = IMU.readACCz()
    
    
    yG2 = (ACCx2 * 0.244)/1000
    xG2 = (ACCy2 * 0.244)/1000
    zG2 = (ACCz2 * 0.244)/1000
    
    
    yG = (ACCx * 0.244)/1000
    xG = (ACCy * 0.244)/1000
    zG = (ACCz * 0.244)/1000
    
    
    ISAccx = xG * 9.81              #VARIABLES TO PRINT IN GUI
    ISAccy = yG * 9.81
    ISAccz = zG * 9.81
    
        
    if 0:
        outputString +=("Acc X = %fG  Acc Y =   %fG  Acc Z =  %fG  " % ( yG2, xG2, zG2)) #Raw acceleration in G
    
    if 0:
        outputString +=("ACC X = %fG  ACC Y =   %fG  ACC Z =  %fG  " % ( yG, xG, zG)) #Filtered Acceleration in G
    
    if 0:
        outputString +=("ACC X = %f m/s^2  ACC Y =   %f m/s^2  ACC Z =  %f m/s^2  " % ( ISAccx, ISAccy, ISAccz)) #Filtered Acceleration in m/s^2

    if 0:                       #Change to '0' to stop showing the angles from the accelerometer
        outputString += "#  ACCX Angle %5.2f ACCY Angle %5.2f  #  " % (AccXangle, AccYangle)

    if 0:                       #Change to '0' to stop  showing the angles from the gyro
        outputString +="\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (gyroXangle,gyroYangle,gyroZangle)

    if 0:                       #Change to '0' to stop  showing the heading
        outputString +="\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)


    #print(outputString)

    #slow program down a bit, makes the output more readable
    time.sleep(0.03)
    
    #distance
    try:
        distanceLow = distanceMethod() * 100 #in cm
    except KeyboardInterrupt:                               #VARIABLE TO PRINT GUI
        GPIO.cleanup()
    #print 'Distance: %3.1f cm' % distanceLow
    #ESCs

    p.ChangeDutyCycle(speed)
    
    
    #Algorithm
    if(distanceLow < 3.0):
        speed += 0.1
    if(distanceLow > 3.0):
        speed -= 0.1
    #safeguards    
    if(distanceLow <= 2.7):
        speed += 1
    if(speed > 12.0):
        speed = 12.0
    if(speed < 9.0):
        speed = 9.0
    if (u==1):
        if (PreviousAccx == 0.0 or PreviousAccy == 0.0 or PreviousAccz == 0.0):
            u = 0
    if (u == 1):
        if ((math.fabs((ISAccx - PreviousAccx) / PreviousAccx)) > 0.05):
            DisplacementX += ISAccx * LP * LP
            if (DisplacementX > 0 and t == 0):
                DisplacementX = 0.0
                t = 1
        if ((math.fabs((ISAccy - PreviousAccy) / PreviousAccy)) > 0.10):
            DisplacementY += ISAccy * LP * LP
            if (DisplacementY < 0 and j == 0):
                DisplacementY = 0.0
                j = 1
        if ((math.fabs((ISAccz - PreviousAccz) / PreviousAccz)) > 0.04):
            DisplacementZ += ISAccz * LP * LP
            if (DisplacementZ < 0 and r == 0):
                DisplacementZ = 0.0
                r = 1
        
    if 0:
        outputString +=("Displacement X = %5.3f m  Displacement Y =   %5.3f m  Displacement Z =  %5.3f m  " % ( DisplacementX, DisplacementY, DisplacementZ)) 

    #print(outputString)

    
    PreviousAccx = ISAccx
    PreviousAccy = ISAccy
    PreviousAccz = ISAccz
    
    u = 1
    
    rpm = (speed/12.0) * 100
    
    #GUI
    displaytable_content(temperature,pressure,ISAccx,ISAccy,ISAccz,DisplacementX,DisplacementY,DisplacementZ,distanceLow,gyroXangle,gyroYangle,gyroZangle,rpm)
    window.update_idletasks()
    window.update()

#******************* END WHILE LOOP*******************************

