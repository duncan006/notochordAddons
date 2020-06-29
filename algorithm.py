#!/usr/bin/env python3

from scipy import integrate
from scipy.signal import savgol_filter,butter
from multiprocessing import Process,Queue,Pipe
import numpy
import time
import activate
import socket
import json

#----------------------USER INPUT----------------------
mass = None #kg
height = None #cm
averagingSeverity = 10 #severity of averaging functions

debugging_output = False
detectWindow = .5

PORT = 30001
IP = "localhost"


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

Mb = 0.678 * mass #Everything waist up including head, arms
Mt = 0.1 * mass 
Ms = 0.0465 * mass 
Mf = 0.0145 * mass 

M = M1 + Mt
Ma = (M1 * a_s) + (Mt * Lt)

M1 = Ms + Mf
M2 = Mb + ( Mt * 2 ) + Ms + Mf
Msum = M1 + M2

c_s = 0.433 * Ls  #measured top down (proximal)

a_s = 0.567 * Ls  #measured bottom up (distal)
a_t = Lt
#a_t = 0.567 * Mt


#----------------------NECESSARY VARIABLE DECLARATIONS----------------------
d_q_s_arr = []
d_q_t_arr = []
d_q_h_arr = []
dd_q_s_arr = []
dd_q_t_arr = []
dd_q_h_arr = []

d_x_p_arr = []
d_x_h_arr = []
dd_x_h_arr = []
dd_x_p_arr = []
timeArray = []

dd_q_s = 0
dd_q_t = 0
dd_q_h = 0
gait = 2
# 0: Heel Strike
# 1: Heel Off
# 2: Toe Off

timeStart = time.time() #use this value (plus a date if its not included) to create a filename for results.

#filepkk = f"pkk{str(time.time())}"
filexkk = f"xkk{str(time.time())}"

#fpkk = open(filepkk,"w+")
fxkk = open(filexkk,"w+")

Q = numpy.array([[0.1*1000,0,0,0],[0,0.1*10,0,0],[0,0,0.1/10,0],[0,0,0,0.1/10]])
                 
R = numpy.array([[25/10,0,0], [0,0.00001*5, 0],[0,0,0.0001*5]])


#----------------------Initialise Socket----------------------
serv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

serv.bind((IP, PORT))
serv.listen(5)
conn, addr = serv.accept()


#--------------------------------------------------------------
#----------------------Debugging Function----------------------
def dbg(name, var):
	if debugging_output = True:
		print(f"{name}: {var}")


#----------------------Arduino Activate Function----------------------
def ardno(msg):
	import serial
	import time

	ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
	ser.flush()
	ser.write(b"{}".format(msg))
   


#----------------------------------------------------------------
#----------------------LOOP/ALGORITHM START----------------------
while True:
    
#----------------------PULLING DATA----------------------

    data = conn.recv(4096)
    if not data: break
    data = json.loads(data.decode())
    
    dataType = data[0]
    thigh = data[1]
    shank = data[2]
    heel = data[3]
    lowBack = data[4]
    
    if dataType = 'r':            
        dd_x_t = thigh[1]
        dd_y_t = thigh[2]
        d_q_t = thigh[0]

        dd_x_s = shank[1]
        dd_y_s = shank[2]
        d_q_s = shank[0]

        dd_x_h = foot[4]
        d_q_h = foot[3]

        dd_x_p = lowBack[4]
    
    if dataType = 'q':
        q_s = shank[0]
        q_t = thigh[0]
        q_h = heel[0]
    
#----------------------SIGNAL FILTERING----------------------        
    
    
    
#----------------------CALCULATING DX----------------------
    #dx is calculated over the last X iterations of the loop
    timeArray.append(time.time())
    while len(timeArray) > averagingSeverity:
        timeArray.pop(0)
        
    if len(timeArray) = 1:
        dx = 0
    else:
        dx = timeArray[len(timeArray) - 1] - timeArray[0]
        deltaT = dx/len(timeArray)
        
    dbg("dx",dx)
    dbg("deltaT",deltaT)


#----------------------CALCULATING ANGULAR ACCELERATIONS----------------------
    #Distance between measurements for the last X iterations are averaged, then divided by the time needed.
    #Heavily bump up averaging severity. Use function of averaging severity as window size for savgol function
    
    d_q_s_arr.append(d_q_s)
    d_q_t_arr.append(d_q_t)
    d_q_h_arr.append(d_q_h)

    while len(d_q_s_arr) > averagingSeverity:
        d_q_s_arr.pop(0)
    while len(d_q_t_arr) > averagingSeverity:
        d_q_t_arr.pop(0)
    while len(d_q_h_arr) > averagingSeverity:
        d_q_h_arr.pop(0)
        
    dd_q_s_arr = numpy.diff(d_q_s_arr)
    dd_q_t_arr = numpy.diff(d_q_t_arr)
    dd_q_h_arr = numpy.diff(d_q_h_arr)

    dd_q_s_old = dd_q_s
    dd_q_t_old = dd_q_t
    dd_q_h_old = dd_q_h

    dd_q_s = sum(dd_q_s_arr) / (len(dd_q_s_arr) * deltaT)
    dd_q_t = sum(dd_q_t_arr) / (len(dd_q_t_arr) * deltaT)
    dd_q_h = sum(dd_q_h_arr) / (len(dd_q_h_arr) * deltaT)
    
    #Using Savitzky-Golay Filter, implement for velocities as well? Could use .butter() instead

    #dd_q_s = scipy.signal.savgol_filter(d_q_s_arr, 10, deriv=1, delta=deltaT)
    #dd_q_t = scipy.signal.savgol_filter(d_q_t_arr, 10, deriv=1, delta=deltaT)

    dbg("dd_q_s",dd_q_s)
    dbg("dd_q_t",dd_q_t)
    
    
    
    
#----------------------CALCULATING VELOCITIES----------------------
    
    dd_x_h_arr.append(dd_x_h)
    dd_x_p_arr.append(dd_x_p)

    while len(dd_x_h_arr) > averagingSeverity:
        dd_x_h_arr.pop(0)
    while len(dd_x_p_arr) > averagingSeverity:
        dd_x_p_arr.pop(0)

    d_x_h_arr = scipy.integrate.cumtrapz(dd_x_h_arr, timeArray) #todo: zero time array
    d_x_p_arr = scipy.integrate.cumtrapz(dd_x_p_arr, timeArray)

    d_x_h = sum(dd_x_h_arr) / (len(ddd_x_h_arr) * deltaT)
    d_x_p = sum(ddd_x_p_arr) / (len(ddd_x_p_arr) * deltaT)

    dbg("d_x_h",d_x_h)
    dbg("d_x_p",d_x_p)




#----------------------SLIP INDICATOR CALCULATIONS----------------------
    Xs1 = Ma * (((dd_q_s ^2) * sin(q_s)) - (der_gy_s * cos(q_s))
    Xs2 = Mt * Lt * (((dd_q_t ^2) * sin(q_t)) - (der_gy_t * cos(q_t)))
    Xs = (Xs1 + Xs2) / M

    dbg("Xs",Xs)

    x_hh = (Lt * sin(q_t)) + (Ls * sin(q_s))
    y_hh = (Lt * cos(q_t)) + (Ls * cos(q_s))
    L_hh = sqrt((x_hh^2) + (y_hh^2))

    dd_q_hh = (dd_y_t - dd_x_h) / L_hh

    slip_indicator = Xs / (beta ^ (dd_q_hh - gamma)))

    dbg("slip_indicator", slip_indicator)





#----------------------SLIP INDICATOR LOGIC----------------------
    #also checks that algorithm has been running for at least five seconds before possible activation.
    #This ensures that no errors will occur due to startup values before the algorithm obtains a moving average.
    if slip_indicator >= slip_constant and time.time() - timeStart > 5:
        trkov_slip_flag = True
    else:
        trkov_slip_flag = False




    
#-----------------------------------------------------------------
#----------------------HEEL STRIKE DETECTION----------------------
    #Detects bottom of downward spike in angular acceleration of the shank in sagittal plane
    #IMU positioned on the outside of the shin
    #TODO determine threshhold value dynamically (using 2/3 of a normalization value through filtering)
    threshStrike = -50
    
    if d_q_s < threshStrike and dd_q_s > 0 and dd_q_s_old <= 0 and gait = 2:
        gait = 0
        
        
        
#--------------------------------------------------------------
#----------------------HEEL OFF DETECTION----------------------
    #Detects top of upward spike in angular acceleration of the heel in sagittal plane
    #IMU positioned on the inside of the heel
    #TODO determine threshhold value dynamically
        #start with a small threshhold, record each peak value, use peak value to modify the threshhold value that converges, with a variable deviation (ex. half? 3/4?)
    threshOff = 50        
    
    if d_q_h > threshOff and dd_q_h < 0 and dd_q_h_old >= 0 and gait = 0:
        gait = 1
        

#--------------------------------------------------------------
#----------------------TOE OFF DETECTION-----------------------
    #Detects return to zero after a downward spike in gyroscopic value in sagittal plane
    #IMU positioned on the inside of the heel
    #TODO use dynamic threshhold value to determine when peak is hit and, subsequently, when zero is hit.
    threshToe = 50        
    
    if d_q_h < threshOff and dd_q_h > 0 and dd_q_h_old <= 0 and gait = 1:
        gait = 2


#-----------------------------------------------------------------
#----------------------SYSTEM ACTIVATION--------------------------		
    #Functions imported from ./activate.py
    dbg("gait", gait)
    if trkov_slip_flag and gait == 0 and time.time() - time_activated > 5:
        ardno("ac")
        time_activated = time.time()
        print("activated")
        
#-------------------------------------------------------------------
#----------------------EKF - SLIP START VALUES----------------------

        q_s_Start = q_s
        q_t_Start = q_t
        time_Start = time.time()
        x_h = 0
        #fpkk.write("SLIP START DETECTED")
        #fpkk.write('\n')
        fxkk.write("SLIP START DETECTED")
        fxkk.write('\n') 
        

#----------------------EKF CALCULATIONS----------------------
    if gait == 0 and time.time() - time_activated < 5:
        Y = numpy.array([dd_x_h, d_x_h, x_h])
        xkk = numpy.array([[x_h], [d_x_h], [q_s], [q_t]])
        pkk = numpy.zeros(4, 4)

        x_h = xkk(1)
        d_x_h = xkk(2)
        q_s = xkk(3)
        q_t = xkk(4)

        Ftheta23 = ( M1 * a_s + M2 * L2 ) / Msum * ( cos(q_s) * d_q_s^2 + dd_q_s * sin(q_s) )
        Ftheta24 = ( M2 * L2 ) / Msum * ( cos(q_t) * d_q_t^2 + dd_q_t * sin(q_t))

        Ftheta = numpy.array([[0,1,0,0],[0,0,Ftheta23,Ftheta24],[0,0,0, 0],[0,0,0,0]])

        F = numpy.zeros(4, 4) + deltaT * Ftheta

        xkk_1 = xkk + deltaT * numpy.array([[d_x_h],[dd_x_h],[d_q_s],[d_q_t]])
        pkk_1 = F * pkk * numpy.transpose(F) + Q

        x_h = xkk_1(1)
        d_x_h = xkk_1(2)
        q_s = xkk_1(3)
        q_t = xkk_1(4)
    
    #-------------------------------------------------------------#

        h11 = 0
        h12 = 0
        h13 = ( M1 * a_s + M2 * L2 ) / Msum * ( cos(q_s) * d_q_s^2 + dd_q_s * sin(q_s) )
        h14 = ( M2 * L2 ) / Msum * ( cos(q_t) * d_q_t^2 + dd_q_t * sin(q_t) )

        h21 = 0
        h22 = -1
        h23 = L1 * d_q_s * sin(q_s)
        h24 = L2 * d_q_t * sin(q_t)

        h31 = -1
        h32 = 0
        h33 = -L1 * cos(q_s)
        h34 = -L2 * cos(q_t)

        H = numpy.array([[h11, h12, h13, h14],[h21, h22, h23, h24],[h31, h32, h33, h34]])

    #-------------------------------------------------------------#
    
        dd_x_h_kin = ( sin(q_s) * ( M1 * a_s + L1 * M2 ) * d_q_s^2 + M2 * a_t * sin(q_t) * d_q_t^2 - dd_q_s * cos(q_s) * ( M1 * a_s + L1 * M2 ) - M2 * a_t * dd_q_t * cos(q_t) ) / (M1 + M2);
        d_x_h_kin = d_x_hip - L1 * d_q_1 * cos(q_s) - L2 * d_q_t * cos(q_t) - d_x_h;
        x_h_kin = d_x_hip * deltaT * (time.time() - time_Start) - L1 * sin(q_s) - L2 * sin(q_t) - ( -L1 * sin(q_s_Start) - L2 * sin(q_t_Start)) - x_h;

        Yhat = numpy.transpose(Y) - numpy.array[ [dd_x_h_kin],[d_x_h_kin],[x_h_kin] ]

        S = H * pkk_1 * numpy.transpose(H) + R
        K = pkk_1 * numpy.transpose(H) / S

        xkk = xkk_1 + K * Yhat
        pkk = ( numpy.zeros(4, 4) - K * H ) * pkk_1

        #fpkk.write(pkk)
        #fpkk.write('\n')
        fxkk.write(xkk)
        fxkk.write('\n')
    elif time.time() - time_activated > 5
        fxkk.write("END OF SLIP")

