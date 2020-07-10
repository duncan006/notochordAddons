#!/usr/bin/env python3
from pythonosc.osc_server import AsyncIOOSCUDPServer
from pythonosc.dispatcher import Dispatcher
import asyncio
import serial
import time
from math import sin, cos, sqrt
from scipy import integrate
from scipy.spatial.transform import Rotation as R
import numpy as np

global varType, dataDict, flagDict, qDataDict, qFlagDict, trueFlagDict, packetReady, passToAlgorithm, qPacketReady, ePacketReady
global startTime, dd_x_t_std_arr, outputVal_arr, kalman_dd_x_t

packetReady = False
ePacketReady = False
qPacketReady = False


ip = "10.0.0.34"
port = 6565
calibTime = 2

#Variable Initializations

trueFlagDict = {
	"rThigh": True,
	"rShank": True,
	"rHeel": True,
	"lowBack": True,
}

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
	"10": "rThigh",
	"11": "rShank",
	"12": "rHeel",
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
    
    r = R.from_quat([x,y,z,w])
    e = r.as_euler('xyz', degrees=False)
    
    #out.append(e[0]) #x-axis euler
    #out.append(e[1]) #y-axis euler
    out.append(e[2]) #z-axis euler
    
    return out


def address_handler(address, *args):
    global varType
    varType = address[13]


def data_handler(address, *args):
    global varType, dataDict, flagDict, qDataDict, qFlagDict, trueFlagDict, packetReady, passToAlgorithm, qPacketReady, ePacketReady
    addr = ''
    addr += str(address[9])
    addr += str(address[11])
    out = []
    
    if addr in addressDict:
        limb = addressDict[addr]
        if varType == "q":
            qDataDict[limb] = package_handler_q(args)
            qFlagDict[limb] = True
        if varType == "r":
            dataDict[limb] = package_handler_raw(args)
            flagDict[limb] = True
        
        if qFlagDict == trueFlagDict:
            for x in range(len(qFlagDict) - 1):
                out.append(qDataDict[orderDict[x]])
                    
            for x in qFlagDict:
                qFlagDict[x] = False
            
            qPacketReady = True
        
        if flagDict == trueFlagDict:
            for x in range(len(flagDict) - 1):
                out.append(dataDict[orderDict[x]])
                    
            for x in flagDict:
                flagDict[x] = False
            
            ePacketReady = True
    
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


def default_handler(address, *args):
    out = 'DEFAULT '
    out += str(address)
    out += str(args)
        

def ardno(msg):
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    ser.write(b"{}".format(msg))


async def main_loop():
    global packetReady, passToAlgorithm
    print(passToAlgorithm)
    
    #----------------------USER INPUT----------------------
    mass = 5 #kg
    height = 10 #cm
    averagingSeverity = 10 #severity of averaging functions

    debugging_output = False
    detectWindow = .5
    
    process_variance = 10


    #----------------------TESTED CONSTANTS----------------------
    slip_constant = 2.83 or 1.87
    beta = 2.718
    gamma = -562 or -377 #deg/s


    #----------------------CALCULATED LIMB LENGTHS----------------------
    #David A. Winter, "Biomechanics and Motor Control of Human Movement" (2009)

    Lt = 0.2 * height #to top of thigh
    #Lt = 0.245 * height #to hip
    Ls = 0.246 * height 
    Lh = .039 * height 

    L1 = Ls + Lh
    L2 = Lt
    
    a_s = 0.567 * Ls  #measured bottom up (distal)
    a_t = Lt
    #a_t = 0.567 * Mt

    Mb = 0.678 * mass #Everything waist up including head, arms
    Mt = 0.1 * mass 
    Ms = 0.0465 * mass 
    Mf = 0.0145 * mass 

    M1 = Ms + Mf
    M2 = Mb + ( Mt * 2 ) + Ms + Mf
    
    Ma = (M1 * a_s) + (Mt * Lt)
    M = M1 + Mt
    Msum = M1 + M2

    c_s = 0.433 * Ls  #measured top down (proximal)


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
    d_q_h_std_arr  = []
    dd_x_p_std_arr = []

    timeStart = time.time() #use this value (plus a date if its not included) to create a filename for results.
    time_activated = time.time()

    Q = np.array([[0.1*1000,0,0,0],[0,0.1*10,0,0],[0,0,0.1/10,0],[0,0,0,0.1/10]])
                     
    R = np.array([[25/10,0,0], [0,0.00001*5, 0],[0,0,0.0001*5]])
    
    
    #----------------------MAIN LOOP----------------------
    while True:
        if packetReady == True:
            packetReady = False
            
            print(passToAlgorithm)
        
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
            d_q_h = h_raw[0]

            dd_x_p = b_raw[3]
        
            q_s = s_ang[0]
            q_t = t_ang[0]
            q_h = h_ang[0]
                
                
                
            #----------------------SIGNAL FILTERING----------------------        
            
            
            
            #----------------------CALIBRATION----------------------
            
            if int(time.time()) - int(timeStart) < int(calibTime):
                
                #Angle Calibration
                
                calibAngThigh.append(tan(dd_x_t/dd_y_t))
                calibAngShank.append(tan(dd_x_s/dd_y_s))
                calibAngHeel.append(tan(dd_x_h/dd_y_h))
         
                thighOffset = sum(calibAngThigh) / len(calibAngThigh)
                shankOffset = sum(calibAngThigh) / len(calibAngThigh)
                heelOffset = sum(calibAngThigh) / len(calibAngThigh)
                
                #Kalman Calibration
                
                dd_x_t_std_arr.append(dd_x_t)
                dd_y_t_std_arr.append(dd_y_t)
                d_q_t_std_arr.append(d_q_t)
                dd_x_s_std_arr.append(dd_x_s)
                dd_y_s_std_arr.append(dd_y_s)
                d_q_s_std_arr.append(d_q_s)
                dd_x_h_std_arr.append(dd_x_h)
                d_q_h_std_arr.append(d_q_h)
                dd_x_p_std_arr.append(dd_x_p)
            
            elif int(time.time()) - int(timeStart) >= int(calibTime) and int(time.time()) - int(timeStart) >= int(calibTime) + 0.1:
                
                #Kalman Calibration Calculate and Save
                
                dd_x_t_variance = (numpy.std(dd_x_t_std_arr)) ** 2
                kalman_dd_x_t = KalmanFilter(process_variance, dd_x_t_variance)
                
                dd_y_t_variance = (numpy.std(dd_y_t_std_arr)) ** 2
                kalman_dd_y_t = KalmanFilter(process_variance, dd_y_t_variance)
                
                d_q_t_variance = (numpy.std(d_q_t_std_arr)) ** 2
                kalman_d_q_t = KalmanFilter(process_variance, d_q_t_variance)
                
                dd_x_s_variance = (numpy.std(dd_x_s_std_arr)) ** 2
                kalman_dd_x_s = KalmanFilter(process_variance, dd_x_s_variance)
                
                dd_y_s_variance = (numpy.std(dd_y_s_std_arr)) ** 2
                kalman_dd_y_s = KalmanFilter(process_variance, dd_y_s_variance)
                
                d_q_s_variance = (numpy.std(d_q_s_std_arr)) ** 2
                kalman_d_q_s = KalmanFilter(process_variance, d_q_s_variance)
                
                dd_x_h_variance = (numpy.std(dd_x_h_std_arr)) ** 2
                kalman_dd_x_h = KalmanFilter(process_variance, dd_x_h_variance)
                
                d_q_h_variance = (numpy.std(d_q_h_std_arr)) ** 2
                kalman_d_q_h = KalmanFilter(process_variance, d_q_h_variance)
                
                dd_x_p_variance = (numpy.std(dd_x_p_std_arr)) ** 2
                kalman_dd_x_p = KalmanFilter(process_variance, dd_x_p_variance)
                
                print("calibration complete")
                print(f"Thigh: {thighOffset} | Shank: {shankOffset} | Heel: {heelOffset}")
                
            else:
                
                #----------------------APPLY ANGLE CALIBRATION----------------------
                q_t -= thighOffset
                q_s -= shankOffset
                q_h -= hellOffset
                
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
                
                kalman_d_q_h.input_latest_noisy_measurement(d_q_h)
                d_q_h = kalman_d_q_h.get_latest_estimated_measurement()
                
                kalman_dd_x_p.input_latest_noisy_measurement(dd_x_p)
                dd_x_p = kalman_dd_x_p.get_latest_estimated_measurement()
                
                #----------------------CALCULATING DX----------------------
                #dx is calculated over the last X iterations of the loop
                timeArray.append(time.time())
                while len(timeArray) > averagingSeverity:
                    timeArray.pop(0)
                    
                if len(timeArray) == 1:
                    dx = 1
                    deltaT = 1

                else:
                    dx = timeArray[len(timeArray) - 1] - timeArray[0]
                    deltaT = dx/len(timeArray)
                    
                #dbg("dx",dx)
                #dbg("deltaT",deltaT)
                
                
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
                
                dd_q_s_old = dd_q_s
                dd_q_t_old = dd_q_t
                dd_q_h_old = dd_q_h
                
                try:
                    dd_q_s = sum(dd_q_s_arr) / (len(dd_q_s_arr) * deltaT)
                    dd_q_t = sum(dd_q_t_arr) / (len(dd_q_t_arr) * deltaT)
                    dd_q_h = sum(dd_q_h_arr) / (len(dd_q_h_arr) * deltaT)
                except ZeroDivisionError:
                    dd_q_s = 0
                    dd_q_t = 0
                    dd_q_h = 0
                
                #Using Savitzky-Golay Filter, implement for velocities as well? Could use .butter() instead
                
                #dd_q_s = scipy.signal.savgol_filter(d_q_s_arr, 10, deriv=1, delta=deltaT)
                #dd_q_t = scipy.signal.savgol_filter(d_q_t_arr, 10, deriv=1, delta=deltaT)
                
                #dbg("dd_q_s",dd_q_s)
                #dbg("dd_q_t",dd_q_t)
                
                
                
                
                #----------------------CALCULATING VELOCITIES----------------------
                #dd_x_
                
                dd_x_h_arr.append(dd_x_h)
                dd_x_p_arr.append(dd_x_p)
                
                while len(dd_x_h_arr) > averagingSeverity:
                    dd_x_h_arr.pop(0)
                while len(dd_x_p_arr) > averagingSeverity:
                    dd_x_p_arr.pop(0)
                
                d_x_h_arr = integrate.cumtrapz(dd_x_h_arr, timeArray) #todo: zero time array
                d_x_p_arr = integrate.cumtrapz(dd_x_p_arr, timeArray)
                
                try:
                    d_x_h = sum(d_x_h_arr) / (len(d_x_h_arr) * deltaT)
                    d_x_p = sum(d_x_p_arr) / (len(d_x_p_arr) * deltaT)
                except ZeroDivisionError:
                    d_x_h = 0
                    d_x_p = 0
                    
                #dbg("d_x_h",d_x_h)
                #dbg("d_x_p",d_x_p)
                
                
                
                
                #----------------------SLIP INDICATOR CALCULATIONS----------------------
                #DOUBLE CHECK ALGORITHM BITS IN HERE!
                Xs1 = Ma * (((d_q_s ** 2) * sin(q_s)) - (dd_q_s * cos(q_s)))
                Xs2 = Mt * Lt * (((d_q_t ** 2) * sin(q_t)) - (dd_q_t * cos(q_t)))
                Xs = (Xs1 + Xs2) / M
                
                #dbg("Xs",Xs)
                
                x_hh = (Lt * sin(q_t)) + (Ls * sin(q_s))
                y_hh = (Lt * cos(q_t)) + (Ls * cos(q_s))
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
                threshStrike = -50
                
                if d_q_s < threshStrike and dd_q_s > 0 and dd_q_s_old <= 0 and gait == 2:
                    gait = 0
                    
                    
                    
                #--------------------------------------------------------------
                #----------------------HEEL OFF DETECTION----------------------
                #Detects top of upward spike in angular acceleration of the heel in sagittal plane
                #IMU positioned on the inside of the heel
                #TODO determine threshhold value dynamically
                    #start with a small threshhold, record each peak value, use peak value to modify the threshhold value that converges, with a variable deviation (ex. half? 3/4?)
                threshOff = 50        
                
                if d_q_h > threshOff and dd_q_h < 0 and dd_q_h_old >= 0 and gait == 0:
                    gait = 1
                    
                
                #--------------------------------------------------------------
                #----------------------TOE OFF DETECTION-----------------------
                #Detects return to zero after a downward spike in gyroscopic value in sagittal plane
                #IMU positioned on the inside of the heel
                #TODO use dynamic threshhold value to determine when peak is hit and, subsequently, when zero is hit.
                threshToe = 50        
                
                if d_q_h < threshOff and dd_q_h > 0 and dd_q_h_old <= 0 and gait == 1:
                    gait = 2
                
                
                #-----------------------------------------------------------------
                #----------------------SYSTEM ACTIVATION--------------------------		
                #Functions imported from ./activate.py
                #dbg("gait", gait)
                if trkov_slip_flag and gait == 0:
                    ardno("ac")
                    time_activated = time.time()
                    print("activated")
                    
                #-------------------------------------------------------------------
                #----------------------EKF - SLIP START VALUES----------------------
                
                    q_s_Start = q_s
                    q_t_Start = q_t
                    time_slip_start = time.time()
                    x_h = 0
                    fxkk.write("SLIP START DETECTED")
                    fxkk.write('\n')
                    
                
                #----------------------EKF CALCULATIONS----------------------
                slipTime = 4
                if gait == 0 and time.time() - time_activated < slipTime:
                    Y = np.array([dd_x_h, d_x_h, x_h])
                    xkk = np.array([[x_h], [d_x_h], [q_s], [q_t]])
                    pkk = np.zeros(4, 4)
                
                    x_h = xkk(1)
                    d_x_h = xkk(2)
                    q_s = xkk(3)
                    q_t = xkk(4)
                
                    Ftheta23 = ( M1 * a_s + M2 * L2 ) / Msum * ( cos(q_s) * d_q_s**2 + dd_q_s * sin(q_s) )
                    Ftheta24 = ( M2 * L2 ) / Msum * ( cos(q_t) * d_q_t**2 + dd_q_t * sin(q_t))
                
                    Ftheta = np.array([[0,1,0,0],[0,0,Ftheta23,Ftheta24],[0,0,0, 0],[0,0,0,0]])
                
                    F = np.zeros(4, 4) + deltaT * Ftheta
                
                    xkk_1 = xkk + deltaT * np.array([[d_x_h],[dd_x_h],[d_q_s],[d_q_t]])
                    pkk_1 = F * pkk * np.transpose(F) + Q
                
                    x_h = xkk_1(1)
                    d_x_h = xkk_1(2)
                    q_s = xkk_1(3)
                    q_t = xkk_1(4)
                
                #-------------------------------------------------------------#
                
                    h11 = 0
                    h12 = 0
                    h13 = ( M1 * a_s + M2 * L2 ) / Msum * ( cos(q_s) * d_q_s**2 + dd_q_s * sin(q_s) )
                    h14 = ( M2 * L2 ) / Msum * ( cos(q_t) * d_q_t**2 + dd_q_t * sin(q_t) )
                
                    h21 = 0
                    h22 = -1
                    h23 = L1 * d_q_s * sin(q_s)
                    h24 = L2 * d_q_t * sin(q_t)
                
                    h31 = -1
                    h32 = 0
                    h33 = -L1 * cos(q_s)
                    h34 = -L2 * cos(q_t)
                
                    H = np.array([[h11, h12, h13, h14],[h21, h22, h23, h24],[h31, h32, h33, h34]])
                
                #-------------------------------------------------------------#
                
                    dd_x_h_kin = ( sin(q_s) * ( M1 * a_s + L1 * M2 ) * (d_q_s**2) + M2 * a_t * sin(q_t) * (d_q_t**2) - dd_q_s * cos(q_s) * ( M1 * a_s + L1 * M2 ) - M2 * a_t * dd_q_t * cos(q_t) ) / (M1 + M2);
                    d_x_h_kin = d_x_hip - L1 * d_q_1 * cos(q_s) - L2 * d_q_t * cos(q_t) - d_x_h;
                    x_h_kin = d_x_hip * deltaT * (time.time() - time_slip_start) - L1 * sin(q_s) - L2 * sin(q_t) - ( -L1 * sin(q_s_Start) - L2 * sin(q_t_Start)) - x_h;
                
                    Yhat = np.transpose(Y) - np.array[ [dd_x_h_kin],[d_x_h_kin],[x_h_kin] ]
                
                    S = H * pkk_1 * np.transpose(H) + R
                    K = pkk_1 * np.transpose(H) / S
                
                    xkk = xkk_1 + K * Yhat
                    pkk = ( np.zeros(4, 4) - K * H ) * pkk_1
                
                    #fpkk.write(pkk)
                    #fpkk.write('\n')
                    #fxkk.write(xkk)
                    #fxkk.write('\n')
                elif time.time() - time_activated > 5:
                    #force end slip after 5 seconds
                    pass
                
        await asyncio.sleep(0)   


async def main_func():
    server = AsyncIOOSCUDPServer((ip, port), dispatcher, asyncio.get_event_loop())
    transport, protocol = await server.create_serve_endpoint()  # Create datagram endpoint and start serving

    await main_loop()  # Enter main loop of program

    transport.close()  # Clean up serve endpoint


dispatcher = Dispatcher()
dispatcher.map("/%%/*", address_handler)
dispatcher.map("/%/*", data_handler)
dispatcher.set_default_handler(default_handler)
asyncio.run(main_func())
