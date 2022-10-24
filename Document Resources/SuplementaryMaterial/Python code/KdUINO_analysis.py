# -*- coding: utf-8 -*-
"""
  KdUINO analysis code 
  by: Raul Bardaji Benach.
  Date: 21/01/2016
  Number of sensors: 4
    
  This work is licensed under the Creative Commons Attribution 
  4.0 International License. To view a copy of this license, 
  visit http://creativecommons.org/licenses/by/4.0/.
"""
import numpy as np
import matplotlib.dates as mdates
from scipy import stats
import sys



def readDatalog(pathDatalog):
    """ 
    Take data from the datalog file.
    Input: The path where the datalog file is placed.
    Return: list with dates of the measurements, list of measurements from sensor 1
        list of meausrements from sensor 2,
        list of measurements from sensor 3,
        list of measurements from sensor 4.
    """
    try:
        data,d1,value1,d2,value2,d3,value3,d4,value4 = np.loadtxt(pathDatalog,unpack=True,converters={ 0: mdates.strpdate2num('%Y/%m/%d_%H:%M:%S')})
        return data,value1,value2,value3,value4
    except:
        return None,None,None,None,None


def readCalibration(pathCalibration):
    """ 
    Take data from the calibration file.
    Input: The path where the calibration file is placed.
    Return: calibration value from sensor 1
        calibration value from sensor 2,
        calibration value from sensor 3,
        calibration value from sensor 4.
    """
    
    try:
        d1,cal1,d2,cal2,d3,cal3,d4,cal4 = np.loadtxt(pathCalibration)
    except: #There is not any calibration file
        cal1=cal2=cal3=cal4 = None
    
    return cal1,cal2,cal3,cal4


def calibration(v1,v2,v3,v4,cal1,cal2,cal3,cal4):
    """ 
    Apply the calibration to the sensor values.
    Input: list of measurements from sensor 1
        list of meausrements from sensor 2,
        list of measurements from sensor 3,
        list of measurements from sensor 4,
        calibration value from sensor 1
        calibration value from sensor 2,
        calibration value from sensor 3,
        calibration value from sensor 4.
    Output: list of calibrated measurements from sensor 1
        list of calibrated meausrements from sensor 2,
        list of calibrated measurements from sensor 3,
        list of calibrated measurements from sensor 4.
    """
    
    # Calculation of the maximum value
    max_value = np.max((cal1,cal2,cal3,cal4))
    # Calculation of the calibration ratios
    ratio1 = max_value/cal1
    ratio2 = max_value/cal2
    ratio3 = max_value/cal3
    ratio4 = max_value/cal4
    
    v1Calibrated = np.multiply(v1,ratio1)
    v2Calibrated = np.multiply(v2,ratio2)
    v3Calibrated = np.multiply(v3,ratio3)
    v4Calibrated = np.multiply(v4,ratio4)
    
    return v1Calibrated,v2Calibrated,v3Calibrated,v4Calibrated

def analysis(depth,date,v1Calibrated,v2Calibrated,v3Calibrated,v4Calibrated):
    """ 
    Calculation of Kd.
    Input: list of depths of each sensor,
        list of dates of measures,
        list of calibrated measurements from sensor 1
        list of calibrated meausrements from sensor 2,
        list of calibrated measurements from sensor 3,
        list of calibrated measurements from sensor 4.
    Output: print of date, kd and r2
        list of kd
        list of r2
    
    """
    r2 = []
    kd = []
    for i in range(0,len(v1)):
        measures = [v1Calibrated[i],v2Calibrated[i],v3Calibrated[i],v4Calibrated[i]]
        ValidDepth,ValidMeasures = errorDetector(depth,measures)
        # Calculation of the linear regression
        slope, intercept, r_value, p_value, std_err = stats.linregress(ValidDepth,np.log(ValidMeasures))
        r2.append(r_value**2)
        kd.append(-slope)
        # Kd is -slope of the linear regression
        print str(mdates.num2date(date[i]).strftime('%Y/%m/%d %H:%M:%S')) + " kd: " + str(-slope) + " r2: " + str(r_value**2) +" Number of good sensors: " + str(len(ValidDepth))
        
    return kd,r2
def errorDetector(Depth,measures):
    """ Return the correct measurement values """
    
    ValidMeasures = []
    ValidDepth = []
    
    if (len(measures) != 0):
        
        # Error detection 1: First sensor is the nearest to the surface, 
        # so it must has the highest value
        if (measures[0] == np.max(measures)):
            ValidDepth.append(Depth[0])
            ValidMeasures.append(measures[0])
        
        #Error detection 2: Sensor 2 has a higher value than sensor
        # 3 because is nearer to the surface 
        if((measures[1] > measures[2]) and (measures[1] > measures[3])):
            ValidDepth.append(Depth[1])
            ValidMeasures.append(measures[1])
        
        # Error detection 3: Sensor 3 has a higher value than sensor
        # 4 because is nearer to the surface 
        if(measures[2] > measures[3]):
            ValidDepth.append(Depth[2])
            ValidMeasures.append(measures[2])      
          
        # Error detection 4: Sensor 4 has a value different to 0 
        if(measures[3] != 0):
            ValidDepth.append(Depth[3])
            ValidMeasures.append(measures[3])
    
    ValidMeasures = np.array(ValidMeasures)
    ValidDepth = np.array(ValidDepth)
    
    return ValidDepth,ValidMeasures

if __name__ == "__main__":
    
    # Enter datalog file path
    pathDatalog = raw_input("Enter the path of the datalog file: ")
    if pathDatalog == "":
        # Default path of the datalog file 
        pathDatalog = "..\\Example of files from the KdUINO\\datalog_example.TXT"
    
    # Read the datalog file
    date,v1,v2,v3,v4 = readDatalog(pathDatalog)
    if date is None:
        print "Datalog file not found"
        exit()
        
    # Enter calibration file path
    pathDatalog = raw_input("Enter the path of the calibration file: ")
    if pathDatalog == "":
        # Default path of the calibration file 
        pathDatalog = "..\\Example of files from the KdUINO\\calibration_example.TXT"
    
    # Read the calibration file
    cal1,cal2,cal3,cal4 = readCalibration(pathDatalog)
    if cal1 is None:
        print "Calibration file not found"
        exit()
    
    # Apply the calibration
    v1Calibrated,v2Calibrated,v3Calibrated,v4Calibrated = calibration(v1,v2,v3,v4,cal1,cal2,cal3,cal4)
    
    depth = []
    for i in range (0,4):
        # Input depth of each sensor in meters
        depth.append(raw_input("Depth of sensor %d: "%(i+1)))     
    try:
        depth = map(float,depth)
    except:
        print "Error: You have to enter NUMBERS."
        exit()
    
    # Obtain kd and r2
    kd,r2 = analysis(depth,date,v1Calibrated,v2Calibrated,v3Calibrated,v4Calibrated)
