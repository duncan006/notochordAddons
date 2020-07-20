#!/usr/bin/env python3
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
import serial
import time
from math import sin, cos, sqrt, tan
from scipy import integrate
from scipy.spatial.transform import Rotation
import numpy as np

global timeStart, calibTime, varType
global dataDict, flagDict, qDataDict, qFlagDict, toggleFlagDict
global packetReady, qPacketReady, ePacketReady, passToAlgorithm
global xkk_arr, pkk_arr
global kalman_dd_x_t, kalman_dd_y_t, kalman_d_q_t, kalman_dd_x, kalman_dd_y_s, kalman_d_q_s, kalman_dd_x_h, kalman_dd_y_h, kalman_d_q_h, kalman_dd_x_p
global dd_x_t_std_arr, dd_y_t_std_arr, d_q_t_std_arr, dd_x_s_std_arr, dd_y_s_std_arr, d_q_s_std_arr, dd_x_h_std_arr, dd_y_h_std_arr, d_q_h_std_arr, dd_x_p_std_arr
global calibAngHeel, calibAngShank, calibAngThigh, thighOffset, shankOffset, heelOffset
global winters
global d_q_s_arr, d_q_t_arr, d_q_h_arr, dd_q_s_arr, dd_q_t_arr, dd_q_h_arr, d_x_p_arr, d_x_h_arr, dd_x_h_arr, dd_x_p_arr
global timeArray, gait, R


ip = "localhost"
port = 6565

calibTime = 2

mass = 5 #kg
height = 10 #cm

process_variance = 10
averagingSeverity = 10

toggleFlagDict = {
	"rThigh": True,
	"rShank": True,
	"rHeel": True,
	"lowBack": True,
}


#Variable Initializations

packetReady = False
ePacketReady = False
qPacketReady = False

dataDict = {
	"rThigh":  [],
	"rShank":  [],
	"rHeel":  [],
	"lowBack":  [],
}

qDataDict = {
	"rThigh":  [],
	"rShank":  [],
	"rHeel":  [],
	"lowBack":  [],
}

flagDict = {
	"rThigh": False,
	"rShank": False,
	"rHeel": False,
    "lowBack": False,
}

qFlagDict = {
	"rThigh": False,
	"rShank": False,
	"rHeel": False,
    "lowBack": False,
}

addressDict = {
	"12": "rThigh",
	"11": "rShank",
	"10": "rHeel",
	"20": "lowBack",
}

orderDict = {
	0: "rThigh",
	1: "rShank",
	2: "rHeel",
	3: "lowBack",
}

passToAlgorithm = {
	"t_raw": [],
	"s_raw": [],
	"h_raw": [],
	"b_raw": [],
    "t_ang": [],
    "s_ang": [],
    "h_ang": [],
}


#-----------------------------------------------------------------
#----------------------NECESSARY VARIABLE DECLARATIONS----------------------
d_q_s_arr  = []
d_q_t_arr  = []
d_q_h_arr  = []
dd_q_s_arr = []
dd_q_t_arr = []
dd_q_h_arr = []

d_x_p_arr  = []
d_x_h_arr  = []
dd_x_h_arr = []
dd_x_p_arr = []
timeArray  = []

dd_q_s = 0
dd_q_t = 0
dd_q_h = 0
gait = 2
# 0: Heel Strike
# 1: Heel Off
# 2: Toe Off

calibAngThigh = []
calibAngShank = []
calibAngHeel  = []

dd_x_t_std_arr = []
dd_y_t_std_arr = []
d_q_t_std_arr  = []
dd_x_s_std_arr = []
dd_y_s_std_arr = []
d_q_s_std_arr  = []
dd_x_h_std_arr = []
dd_y_h_std_arr = []
d_q_h_std_arr  = []
dd_x_p_std_arr = []

timeStart = time.time() #use this value (plus a date if its not included) to create a filename for results.
time_slip_start = time.time()

Q = np.array([[0.1*1000,0,0,0],[0,0.1*10,0,0],[0,0,0.1/10,0],[0,0,0,0.1/10]])
                 
R = np.array([[25/10,0,0], [0,0.00001*5, 0],[0,0,0.0001*5]])

fileDump = open("algDump.txt", "w+")
fileOGDump = open("algDump_nofilter.txt", "w+")
fileDump.write("q_s | d_q_s | dd_x_s | dd_y_s | q_h | d_q_h | dd_x_h | dd_y_h")
fileOGDump.write("q_s | d_q_s | dd_x_s | dd_y_s | q_h | d_q_h | dd_x_h | dd_y_h")


class KalmanFilter(object):

    def __init__(self, process_variance, estimated_measurement_variance):
        self.process_variance = process_variance
        self.estimated_measurement_variance = estimated_measurement_variance
        self.posteri_estimate = 0.0
        self.posteri_error_estimate = 1.0

    def input_latest_noisy_measurement(self, measurement):
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.posteri_error_estimate + self.process_variance

        blending_factor = priori_error_estimate / (priori_error_estimate + self.estimated_measurement_variance)
        self.posteri_estimate = priori_estimate + blending_factor * (measurement - priori_estimate)
        self.posteri_error_estimate = (1 - blending_factor) * priori_error_estimate

    def get_latest_estimated_measurement(self):
        return self.posteri_estimate


class unwrapper:
    def __init__(self, lastInput):
        self.lastInput = lastInput
        self.calib = 0
        
    def unwrap(self, nextInput):
        diff = self.lastInput - nextInput
    
        if diff > 3:
            self.calib = self.calib - (2 * math.pi)
    
        elif diff < -3:
            self.calib = self.calib + (2 * math.pi)
    
        outputVar = nextInput - self.calib
        self.lastInput = nextInput
        
        return outputVar


def bodyParameters(mass, height):
    global winters
    winters = {}

    #David A. Winter, "Biomechanics and Motor Control of Human Movement" (2009)

    winters["Lt"] = 0.2 * height #to top of thigh
    #winters["Lt"] = 0.245 * height #to hip
    winters["Ls"] = 0.246 * height 
    winters["Lh"] = .039 * height 

    winters["L1"] = winters["Ls"] + winters["Lh"]
    winters["L2"] = winters["Lt"]
    
    winters["a_s"] = 0.567 * winters["Ls"]  #measured bottom up (distal)
    winters["a_t"] = winters["Lt"]
    #winters["a_t"] = 0.567 * winters["Mt"]

    winters["Mb"] = 0.678 * mass #Everything waist up including head, arms
    winters["Mt"] = 0.1 * mass 
    winters["Ms"] = 0.0465 * mass 
    winters["Mf"] = 0.0145 * mass 

    winters["M1"] = winters["Ms"]+ winters["Mf"]
    winters["M2"] = winters["Mb"] + ( winters["Mt"] * 2 ) + winters["Ms"]+ winters["Mf"]
    
    winters["Ma"] = (winters["M1"] * winters["a_s"]) + (winters["Mt"] * winters["Lt"])
    winters["M"] = winters["M1"] + winters["Mt"]
    winters["Msum"] = winters["M1"] + winters["M2"]

    winters["c_s"] = 0.433 * winters["Ls"]  #measured top down (proximal)


def package_handler_raw(tup):
    
    out = []
    
    #out.append(tup[0])    #gy_x
    #out.append(tup[1])    #gy_y   
    out.append(tup[2])    #gy_z
    out.append(tup[3])    #ac_x
    out.append(tup[4])    #ac_y
    out.append(tup[5])    #ac_z
    
    return out


def package_handler_q(tup):
    keepX = False
    keepY = False
    keepZ = True
    
    out = []
    w = tup[0]
    x = tup[1]
    y = tup[2]
    z = tup[3]
    
    r = Rotation.from_quat([x,y,z,w])
    e = r.as_euler('xyz', degrees=False)
    
    #out.append(e[0]) #x-axis euler
    #out.append(e[1]) #y-axis euler
    out.append(e[2]) #z-axis euler
    
    return out


def address_handler(address, *args):
    global varType
    varType = address[13]


def data_handler(address, *args):
    global timeStart, calibTime, varType
    global dataDict, flagDict, qDataDict, qFlagDict, toggleFlagDict
    global packetReady, qPacketReady, ePacketReady, passToAlgorithm
    global xkk_arr, pkk_arr
    global kalman_dd_x_t, kalman_dd_y_t, kalman_d_q_t, kalman_dd_x_s, kalman_dd_y_s, kalman_d_q_s, kalman_dd_x_h, kalman_dd_y_h, kalman_d_q_h, kalman_dd_x_p
    global dd_x_t_std_arr, dd_y_t_std_arr, d_q_t_std_arr, dd_x_s_std_arr, dd_y_s_std_arr, d_q_s_std_arr, dd_x_h_std_arr, dd_y_h_std_arr, d_q_h_std_arr, dd_x_p_std_arr
    global calibAngHeel, calibAngShank, calibAngThigh, thighOffset, shankOffset, heelOffset
    global winters
    global d_q_s_arr, d_q_t_arr, d_q_h_arr, dd_q_s_arr, dd_q_t_arr, dd_q_h_arr, d_x_p_arr, d_x_h_arr, dd_x_h_arr, dd_x_p_arr
    global timeArray, gait, R
    
    
    #-----------------------------------------------------------------
    #----------------------TESTED CONSTANTS----------------------
    slip_constant = 2.83 or 1.87
    beta = 2.718
    gamma = -562 or -377 #deg/s
    
    #--------------------------------------------------------------
    
    addr = ''
    addr += str(address[9])
    addr += str(address[11])
    out = []
    
    
    #Takes in individual data and assembles into packages
    if addr in addressDict:
        limb = addressDict[addr]
        if varType == "q":
            qDataDict[limb] = package_handler_q(args)
            qFlagDict[limb] = True
        if varType == "r":
            dataDict[limb] = package_handler_raw(args)
            flagDict[limb] = True
        
        if qFlagDict == toggleFlagDict:
            for x in range(len(qFlagDict) - 1):
                out.append(qDataDict[orderDict[x]])
                    
            for x in qFlagDict:
                qFlagDict[x] = False
            
            qPacketReady = True
        
        if flagDict == toggleFlagDict:
            for x in range(len(flagDict) - 1):
                out.append(dataDict[orderDict[x]])
                    
            for x in flagDict:
                flagDict[x] = False
            
            ePacketReady = True
    
    
    #Assembles full packet for sending to Algorithm
    if qPacketReady and ePacketReady:
        qPacketReady = False
        ePacketReady = False
        passToAlgorithm["t_raw"] = dataDict["rThigh"]
        passToAlgorithm["s_raw"] = dataDict["rShank"]
        passToAlgorithm["h_raw"] = dataDict["rHeel"]
        passToAlgorithm["b_raw"] = dataDict["lowBack"]
        
        passToAlgorithm["t_ang"] = qDataDict["rThigh"]
        passToAlgorithm["s_ang"] = qDataDict["rShank"]
        passToAlgorithm["h_ang"] = qDataDict["rHeel"]
        packetReady = True
        
    #-------------------------------------------------------------#    
    #Code is broken into reader above and algorithm below for increased customization and ease of changing algorithm.
        
    
    if packetReady:
        packetReady = False
        
        #Update data
        t_ang = passToAlgorithm['t_ang']
        s_ang = passToAlgorithm['s_ang']
        h_ang = passToAlgorithm['h_ang']
        t_raw = passToAlgorithm['t_raw']
        s_raw = passToAlgorithm['s_raw']
        h_raw = passToAlgorithm['h_raw']
        b_raw = passToAlgorithm['b_raw']
                 
        dd_x_t = t_raw[1]
        dd_y_t = t_raw[2]
        d_q_t = t_raw[0]

        dd_x_s = s_raw[1]
        dd_y_s = s_raw[2]
        d_q_s = s_raw[0]

        dd_x_h = h_raw[1]
        dd_y_h = h_raw[2]
        d_q_h = h_raw[0]

        dd_x_p = b_raw[3]
        
        q_s = s_ang[0]
        q_t = t_ang[0]
        q_h = h_ang[0]     
        
        fileOGDump.write(f"{q_s} {d_q_s} {dd_x_s} {dd_y_s} {q_h} {d_q_h} {dd_x_h} {dd_y_h}\n")
        print(f"{q_s} {d_q_s} {dd_x_s} {dd_y_s} {q_h} {d_q_h} {dd_x_h} {dd_y_h}")
        
        try:
            deltaT = lastTime - time.time()
        except:
            deltaT = 1
            
        lastTime = time.time()
        
        #-----------------------------------------------------------------
        #----------------------CALIBRATION----------------------
        
        if time.time() - timeStart < calibTime:
            
            fileDump.write(f"{q_s} {d_q_s} {dd_x_s} {dd_y_s} {q_h} {d_q_h} {dd_x_h} {dd_y_h}\n")
            
            #-----------------------------------------------------------------
            #---------------------------Angle Calibration----------------------
            
            calibAngThigh.append(atan2(dd_x_t,dd_y_t))
            calibAngShank.append(atan2(dd_x_s,dd_y_s))
            calibAngHeel.append(atan2(dd_x_h,dd_y_h))
        
            thighOffset = np.mean(calibAngThigh)
            shankOffset = np.mean(calibAngShank)
            heelOffset = np.mean(calibAngHeel)
            
            
            #-----------------------------------------------------------------
            #---------------------Kalman Calibration---------------------
            
            dd_x_t_std_arr.append(dd_x_t)
            dd_y_t_std_arr.append(dd_y_t)
            d_q_t_std_arr.append(d_q_t)
            dd_x_s_std_arr.append(dd_x_s)
            dd_y_s_std_arr.append(dd_y_s)
            d_q_s_std_arr.append(d_q_s)
            dd_x_h_std_arr.append(dd_x_h)
            dd_y_h_std_arr.append(dd_y_h)
            d_q_h_std_arr.append(d_q_h)
            dd_x_p_std_arr.append(dd_x_p)
            
            
            #-----------------------------------------------------------------
            #----------------------Kalman Calibration Calculate and Save---------------
            
        elif time.time() - timeStart >= calibTime and time.time() - timeStart <= calibTime + 0.2:
            
            dd_x_t_variance = (np.std(dd_x_t_std_arr)) ** 2
            kalman_dd_x_t = KalmanFilter(process_variance, dd_x_t_variance)
            
            dd_y_t_variance = (np.std(dd_y_t_std_arr)) ** 2
            kalman_dd_y_t = KalmanFilter(process_variance, dd_y_t_variance)
            
            d_q_t_variance = (np.std(d_q_t_std_arr)) ** 2
            kalman_d_q_t = KalmanFilter(process_variance, d_q_t_variance)
            
            dd_x_s_variance = (np.std(dd_x_s_std_arr)) ** 2
            kalman_dd_x_s = KalmanFilter(process_variance, dd_x_s_variance)
            
            dd_y_s_variance = (np.std(dd_y_s_std_arr)) ** 2
            kalman_dd_y_s = KalmanFilter(process_variance, dd_y_s_variance)
            
            d_q_s_variance = (np.std(d_q_s_std_arr)) ** 2
            kalman_d_q_s = KalmanFilter(process_variance, d_q_s_variance)
            
            dd_x_h_variance = (np.std(dd_x_h_std_arr)) ** 2
            kalman_dd_x_h = KalmanFilter(process_variance, dd_x_h_variance)
            
            dd_y_h_variance = (np.std(dd_y_h_std_arr)) ** 2
            kalman_dd_y_h = KalmanFilter(process_variance, dd_y_h_variance)
            
            d_q_h_variance = (np.std(d_q_h_std_arr)) ** 2
            kalman_d_q_h = KalmanFilter(process_variance, d_q_h_variance)
            
            dd_x_p_variance = (np.std(dd_x_p_std_arr)) ** 2
            kalman_dd_x_p = KalmanFilter(process_variance, dd_x_p_variance)
            
            fileDump.write(f"{q_s} {d_q_s} {dd_x_s} {dd_y_s} {q_h} {d_q_h} {dd_x_h} {dd_y_h} [CALIBRATION COMPLETE]\n")
            print("calibration complete")
            
            unwrapT = unwrapper(q_t)
            unwrapS = unwrapper(q_s)
            unwrapH = unwrapper(q_h)
            
        else: #After calibTime, start running actual algorithm
            
            #-----------------------------------------------------------------
            #----------------------Unwrapping Angles----------------------
            
            q_t = unwrapT.unwrap(q_t)
            q_s = unwrapS.unwrap(q_s)
            q_h = unwrapH.unwrap(q_h)
            
            
            #-----------------------------------------------------------------
            #----------------------APPLY ANGLE CALIBRATION----------------------
            
            q_t -= thighOffset
            q_s -= shankOffset
            q_h -= heelOffset
            
            
            #-----------------------------------------------------------------
            #----------------------APPLY KALMAN FILTER----------------------
            
            kalman_dd_x_t.input_latest_noisy_measurement(dd_x_t)
            dd_x_t = kalman_dd_x_t.get_latest_estimated_measurement()
            
            kalman_dd_y_t.input_latest_noisy_measurement(dd_y_t)
            dd_y_t = kalman_dd_y_t.get_latest_estimated_measurement()
            
            kalman_d_q_t.input_latest_noisy_measurement(d_q_t)
            d_q_t = kalman_d_q_t.get_latest_estimated_measurement()
            
            kalman_dd_x_s.input_latest_noisy_measurement(dd_x_s)
            dd_x_s = kalman_dd_x_s.get_latest_estimated_measurement()
            
            kalman_dd_y_s.input_latest_noisy_measurement(dd_y_s)
            dd_y_s = kalman_dd_y_s.get_latest_estimated_measurement()
            
            kalman_d_q_s.input_latest_noisy_measurement(d_q_s)
            d_q_s = kalman_d_q_s.get_latest_estimated_measurement()
            
            kalman_dd_x_h.input_latest_noisy_measurement(dd_x_h)
            dd_x_h = kalman_dd_x_h.get_latest_estimated_measurement()
            
            kalman_dd_y_h.input_latest_noisy_measurement(dd_y_h)
            dd_y_h = kalman_dd_y_h.get_latest_estimated_measurement()
            
            kalman_d_q_h.input_latest_noisy_measurement(d_q_h)
            d_q_h = kalman_d_q_h.get_latest_estimated_measurement()
            
            kalman_dd_x_p.input_latest_noisy_measurement(dd_x_p)
            dd_x_p = kalman_dd_x_p.get_latest_estimated_measurement()
            
            
            fileDump.write(f"{q_s} {d_q_s} {dd_x_s} {dd_y_s} {q_h} {d_q_h} {dd_x_h} {dd_y_h}\n")
            
            
            #-----------------------------------------------------------------
            #----------------------CALCULATING ANGULAR ACCELERATIONS----------------------
            #Distance between measurements for the last X iterations are averaged, then divided by the time needed.
            #Heavily bump up averaging severity. Use function of averaging severity as window size for savgol function
            #d_q_t --> dd_q_t
            #d_q_s --> dd_q_s
            #d_q_h --> dd_q_h
            
            d_q_s_arr.append(d_q_s)
            d_q_t_arr.append(d_q_t)
            d_q_h_arr.append(d_q_h)
            
            while len(d_q_s_arr) > averagingSeverity:
                d_q_s_arr.pop(0)
            while len(d_q_t_arr) > averagingSeverity:
                d_q_t_arr.pop(0)
            while len(d_q_h_arr) > averagingSeverity:
                d_q_h_arr.pop(0)
                
            dd_q_s_arr = np.diff(d_q_s_arr)
            dd_q_t_arr = np.diff(d_q_t_arr)
            dd_q_h_arr = np.diff(d_q_h_arr)
            
            try:
                dd_q_s = np.mean(dd_q_s_arr)
                dd_q_t = np.mean(dd_q_t_arr)
                dd_q_h = np.mean(dd_q_h_arr)
            except ZeroDivisionError:
                dd_q_s = 0
                dd_q_t = 0
                dd_q_h = 0           
            
            
            
            #-----------------------------------------------------------------
            #----------------------CALCULATING VELOCITIES----------------------
            #dd_x_
            
            dd_x_h_arr.append(dd_x_h)
            dd_x_p_arr.append(dd_x_p)
            
            while len(dd_x_h_arr) > averagingSeverity:
                dd_x_h_arr.pop(0)
            while len(dd_x_p_arr) > averagingSeverity:
                dd_x_p_arr.pop(0)
            
            d_x_h_arr = integrate.cumtrapz(dd_x_h_arr) #todo: timeArray
            d_x_p_arr = integrate.cumtrapz(dd_x_p_arr)
            
            try:
                d_x_h = sum(d_x_h_arr) / (len(d_x_h_arr))
                d_x_p = sum(d_x_p_arr) / (len(d_x_p_arr))
            except ZeroDivisionError:
                d_x_h = 0
                d_x_p = 0
                
            #dbg("d_x_h",d_x_h)
            #dbg("d_x_p",d_x_p)
            
            
            
            
            #-----------------------------------------------------------------
            #----------------------SLIP INDICATOR CALCULATIONS----------------------
            Xs1 = winters["Ma"] * (((d_q_s ** 2) * sin(q_s)) - (dd_q_s * cos(q_s)))
            Xs2 = winters["Mt"] * winters["Lt"] * (((d_q_t ** 2) * sin(q_t)) - (dd_q_t * cos(q_t)))
            Xs = (Xs1 + Xs2) / winters["M"] 
            
            #dbg("Xs",Xs)
            
            x_hh = (winters["Lt"] * sin(q_t)) + (winters["Ls"] * sin(q_s))
            y_hh = (winters["Lt"] * cos(q_t)) + (winters["Ls"] * cos(q_s))
            L_hh = sqrt((x_hh**2) + (y_hh**2))
            
            dd_q_hh = (dd_y_t - dd_x_h) / L_hh
            
            slip_indicator = Xs / (beta ** (dd_q_hh - gamma))
            
            
            print(f"slip_indicator {slip_indicator}")
            
            #dbg("slip_indicator", slip_indicator)
            
            
            
            
            #-----------------------------------------------------------------
            #----------------------SLIP INDICATOR LOGIC----------------------
            #also checks that algorithm has been running until calibration is complete.
            #This ensures that no errors will occur due to startup values before the algorithm obtains a moving average.
            if slip_indicator >= slip_constant and time.time() - timeStart > calibTime:
                trkov_slip_flag = True
            else:
                trkov_slip_flag = False
            
            
            
            
            #-----------------------------------------------------------------
            #----------------------HEEL STRIKE DETECTION----------------------
            #Detects bottom of downward spike in angular acceleration of the shank in sagittal plane
            #IMU positioned on the outside of the shin
            #TODO determine threshhold value dynamically (using 2/3 of a normalization value through filtering)
            #threshStrike = -50
            #
            #if d_q_s < threshStrike and dd_q_s > 0 and dd_q_s_old <= 0 and gait == 2:
            #    gait = 0
                
                
                
                
            #--------------------------------------------------------------
            #----------------------HEEL OFF DETECTION----------------------
            #Detects top of upward spike in angular acceleration of the heel in sagittal plane
            #IMU positioned on the inside of the heel
            #TODO determine threshhold value dynamically
                #start with a small threshhold, record each peak value, use peak value to modify the threshhold value that converges, with a variable deviation (ex. half? 3/4?)
            #threshOff = 50        
            #
            #if d_q_h > threshOff and dd_q_h < 0 and dd_q_h_old >= 0 and gait == 0:
            #    gait = 1
                
            
            
            #--------------------------------------------------------------
            #----------------------TOE OFF DETECTION-----------------------
            #Detects return to zero after a downward spike in gyroscopic value in sagittal plane
            #IMU positioned on the inside of the heel
            #TODO use dynamic threshhold value to determine when peak is hit and, subsequently, when zero is hit.
            #threshToe = 50        
            #
            #if d_q_h < threshOff and dd_q_h > 0 and dd_q_h_old <= 0 and gait == 1:
            #    gait = 2
            
            
            
            
            #-----------------------------------------------------------------
            #----------------------SYSTEM ACTIVATION--------------------------		
            #dbg("gait", gait)
            if trkov_slip_flag and gait == 0:
                ardno("ac")
                print("activated")
                
                
                
                
            #-------------------------------------------------------------------
            #----------------------EKF - SLIP START VALUES----------------------
            #When slip detected, set start values for EKF and start EKF by resetting time_slip_start to current time.
            
                q_s_Start = q_s
                q_t_Start = q_t
                time_slip_start = time.time()
                x_h = 0
                
                xkk_arr = []
                pkk_arr = []

                
                
            #-----------------------------------------------------------------
            #----------------------EKF CALCULATIONS----------------------
            slipTime = 4
            if gait == 0 and time.time() - time_slip_start < slipTime:
                Y = np.array([dd_x_h, d_x_h, x_h])
                xkk = np.array([[x_h], [d_x_h], [q_s], [q_t]])
                pkk = np.zeros(4, 4)
            
                x_h = xkk[0]
                d_x_h = xkk[1]
                q_s = xkk[2]
                q_t = xkk[3]
            
                Ftheta23 = ( winters["M1"] * winters["a_s"] + winters["M2"] * winters["L2"] ) / winters["Msum"] * ( cos(q_s) * d_q_s**2 + dd_q_s * sin(q_s) )
                Ftheta24 = ( winters["M2"] * winters["L2"] ) / winters["Msum"] * ( cos(q_t) * d_q_t**2 + dd_q_t * sin(q_t))
            
                Ftheta = np.array([[0,1,0,0],[0,0,Ftheta23,Ftheta24],[0,0,0, 0],[0,0,0,0]])
            
                F = np.zeros(4, 4) + deltaT * Ftheta
            
                xkk_1 = xkk + deltaT * np.array([[d_x_h],[dd_x_h],[d_q_s],[d_q_t]])
                pkk_1 = F * pkk * np.transpose(F) + Q
            
                x_h = xkk_1[0]
                d_x_h = xkk_1[1]
                q_s = xkk_1[2]
                q_t = xkk_1[3]
            
            #-------------------------------------------------------------#
            
                h11 = 0
                h12 = 0
                h13 = ( winters["M1"] * winters["a_s"] + winters["M2"] * winters["L2"] ) / winters["Msum"] * ( cos(q_s) * d_q_s**2 + dd_q_s * sin(q_s) )
                h14 = ( winters["M2"] * winters["L2"] ) / winters["Msum"] * ( cos(q_t) * d_q_t**2 + dd_q_t * sin(q_t) )
            
                h21 = 0
                h22 = -1
                h23 = winters["L1"] * d_q_s * sin(q_s)
                h24 = winters["L2"] * d_q_t * sin(q_t)
            
                h31 = -1
                h32 = 0
                h33 = -winters["L1"] * cos(q_s)
                h34 = -winters["L2"] * cos(q_t)
            
                H = np.array([[h11, h12, h13, h14],[h21, h22, h23, h24],[h31, h32, h33, h34]])
            
            #-------------------------------------------------------------#
            
                dd_x_h_kin = ( sin(q_s) * ( winters["M1"] * winters["a_s"] + winters["L1"] * winters["M2"] ) * (d_q_s**2) + winters["M2"] * winters["a_t"] * sin(q_t) * (d_q_t**2) - dd_q_s * cos(q_s) * ( winters["M1"] * winters["a_s"] + winters["L1"] * winters["M2"] ) - winters["M2"] * winters["a_t"] * dd_q_t * cos(q_t) ) / (winters["M1"] + winters["M2"]);
                d_x_h_kin = d_x_hip - winters["L1"] * d_q_1 * cos(q_s) - winters["L2"] * d_q_t * cos(q_t) - d_x_h;
                x_h_kin = d_x_hip * deltaT * (time.time() - time_slip_start) - winters["L1"] * sin(q_s) - winters["L2"] * sin(q_t) - ( -winters["L1"] * sin(q_s_Start) - winters["L2"] * sin(q_t_Start)) - x_h;
            
                Yhat = np.transpose(Y) - np.array[ [dd_x_h_kin],[d_x_h_kin],[x_h_kin] ]
            
                S = H * pkk_1 * np.transpose(H) + R
                K = pkk_1 * np.transpose(H) / S
            
                xkk = xkk_1 + K * Yhat
                pkk = ( np.zeros(4, 4) - K * H ) * pkk_1
                
                xkk_arr.append(xkk)
                pkk_arr.append(pkk)
        

def default_handler(address, *args):
    out = 'DEFAULT '
    out += str(address)
    out += str(args)
    print(out)


def ardno(msg):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    ser.write(b"{}".format(msg))
    

def main_func(ip, port):   
    dispatcher = Dispatcher()
    dispatcher.map("/%%/*", address_handler)
    dispatcher.map("/%/*", data_handler)
    dispatcher.set_default_handler(default_handler)
    
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()  # Blocks forever



if __name__ == "__main__":
    bodyParameters(mass, height)
    main_func(ip, port)
    client.close()
