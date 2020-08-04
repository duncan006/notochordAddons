#!/usr/bin/env python3
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
import serial
import time
from math import sin, cos, sqrt, atan2
from scipy import integrate
from scipy.spatial.transform import Rotation
import numpy as np

global timeStart, timeCurrent, varType
global dataDict, flagDict, qDataDict, qFlagDict, toggleFlagDict
global packetReady, rPacketReady, passToAlgorithm
global winters, fileDump
global gaitDetectHeel, gaitDetectShank, gaitDetectThigh



ip = "localhost"
port = 6565

mass = 60 #kg
height = 180 #cm


toggleFlagDict = {
	"rThigh": True,
	"rShank": True,
	"rHeel": True,
	"lowBack": True,
}


#Variable Initializations

packetReady = False
rPacketReady = False
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



class gaitDetect:
    def __init__(self):
        #State Values from Sensors
        self.gyX = 0
        self.gyY = 0
        self.gyZ = 0
        
        self.acX = 0
        self.acY = 0
        self.acZ = 0

        #testVal() variables
        self.firstVar = 0
        self.movingArr = [0]
        self.significance = 0
        self.movingAvgAccuracy = 2
        self.movingAvg = 0
        self.lastAvg = 0
        self.timeLastHeelStrike = 0
        self.timeLastToeOff = 0
        self.gaitStage = 0 #1 for swing, 0 for stance
        self.eventTimer = .1
        self.standing = False
        self.standingLimit = 200 * .07
        self.concurrentZeroes = 0
        self.concurrentZeroesLimit = 5
        
        #angleCalc() variables
        self.timeToRun = 0
        self.timeLastValue = time.time()
        self.currentTime = time.time()
        
        self.zAngleChangeArray = [0]
        self.zAngleChangeArrayLimit = 5
        self.zAngleArray = [0]
        self.zAngleArrayLimit = 50
        self.wasStanding = False
        self.calibVal = .1
        self.zAngle = 0
        
        #angularAcceleration() variables
        self.gyZarray = [0]
        self.angularAcceleration = 0
    
    def angularAcceleration(self):
        self.gyZarray.append(self.gyZ)
        
        while len(self.gyZarray) > 5:
            self.gyZarray.pop(0)
            
        diff_arr = np.diff(self.gyZarray)
        
        try:
            self.angularAcceleration = np.mean(diff_arr)
        except:
            self.angularAcceleration = 0
            
        return self.angularAcceleration

        
    def angleCalc(self, gaitDetectObject):
        #Usually best to run testVal first, then angleCalc
        self.timeLastValue = self.currentTime
        self.currentTime = time.time()
        self.timeToRun = self.currentTime - self.timeLastValue
        
        zAngleChange = self.gyZ * self.timeToRun
        
        self.zAngleChangeArray.append(zAngleChange)
        
        if len(self.zAngleChangeArray) > self.zAngleChangeArrayLimit:
            self.zAngleChangeArray.pop(0)
    
        if gaitDetectObject.standing == False:
            if self.wasStanding == True:
                self.wasStanding = False
                self.zAngle += np.sum(self.zAngleChangeArray)
                self.calibVal = .1
            else:
                self.zAngle += zAngleChange
                #self.calibVal += .001
                if np.mean(self.zAngleArray) > 0:
                    self.zAngle -= self.calibVal
                elif np.mean(self.zAngleArray) < 0:
                    self.zAngle += self.calibVal
                
        elif self.standing == True:
            if self.zAngle > 1:
                self.zAngle -= 1
            elif self.zAngle < -1:
                self.zAngle += 1
            else:
                pass
            self.wasStanding = True
            
        
        self.zAngleArray.append(self.zAngle)
    
        if len(self.zAngleArray) > self.zAngleArrayLimit:
            self.zAngleArray.pop(0)
                
    def testVal(self):
        self.movingArr.append(self.gyZ)
        if len(self.movingArr) > self.movingAvgAccuracy:
            self.movingArr.pop(0)
        self.movingAvg = np.mean(self.movingArr)
        
        if self.standing == True:
            self.gaitStage = 0
            if self.movingAvg < - self.standingLimit or self.movingAvg > self.standingLimit:
                self.standing = False
                self.lastAvg = - self.movingAvg
                
        if self.standing == False:
            if self.movingAvg < self.standingLimit and self.movingAvg > - self.standingLimit:
                self.concurrentZeroes += 1
            else:
                self.concurrentZeroes = 0
                
            if self.concurrentZeroes > self.concurrentZeroesLimit:
                self.standing = True
        
        if self.significance == 0 and not self.standing:
            if self.movingAvg > 0 and self.lastAvg < 0: #detects negative to positive, aka heel strike or start of stance phase
                self.significance = 1
                self.timeLastHeelStrike = time.time()
                self.gaitStage = 0
            elif self.movingAvg < 0 and self.lastAvg > 0: #detects positive to negative, aka toe off or start of swing phase
                self.significance = -1
                self.timeLastToeOff = time.time()
                self.gaitStage = 1
                
        elif self.significance != 0:
            if time.time() - self.timeLastHeelStrike > self.eventTimer and time.time() - self.timeLastToeOff > self.eventTimer:
                self.significance = 0
        
        #Implement other leg IMU - other leg heel strike must occur before measured leg toe off. (and vice versa)

        self.lastAvg = self.movingAvg



def package_handler_raw(tup):
    
    out = []
    
    for x in tup:
        out.append(x)
    
    return out

    
    
def slipAlgorithm(q_s, q_t, d_q_s, d_q_t, dd_q_s, dd_q_t, dd_y_t, dd_x_h):

    slip_constant = 2.83 #or 1.87
    beta = 2.718
    gamma = -562 #or -377 #deg/s
    
    Xs1 = winters["Ma"] * (((d_q_s ** 2) * sin(q_s)) - (dd_q_s * cos(q_s)))
    Xs2 = winters["Mt"] * winters["Lt"] * (((d_q_t ** 2) * sin(q_t)) - (dd_q_t * cos(q_t)))
    Xs = (Xs1 + Xs2) / winters["M"]
    
    x_hh = (winters["Lt"] * sin(q_t)) + (winters["Ls"] * sin(q_s))
    y_hh = (winters["Lt"] * cos(q_t)) + (winters["Ls"] * cos(q_s))
    L_hh = sqrt((x_hh**2) + (y_hh**2))
    
    dd_q_hh = (dd_y_t - dd_x_h) / L_hh
    
    slip_indicator = Xs / (beta ** (dd_q_hh - gamma))
    
    if slip_indicator >= slip_constant:
        return True
    else:
        return False



def data_handler(address, *args):
    global timeStart, timeCurrent, varType
    global dataDict, flagDict, qDataDict, qFlagDict, toggleFlagDict
    global packetReady, rPacketReady, passToAlgorithm
    global winters, fileDump
    global gaitDetectHeel, gaitDetectShank, gaitDetectThigh
    
    try:
        timeLastRun = timeCurrent
        timeCurrent = time.time()
        timeToRun = timeCurrent - timeLastRun
    except:
        timeToRun = .02
    
    
    varType = address[10]
    
    addr = ''
    addr += str(address[len(address) - 3])
    addr += str(address[len(address) - 1])
    
    out = []
       
    
    
    #Takes in individual data and assembles into packages
    if addr in addressDict:
        limb = addressDict[addr]

        if varType == "r":
            dataDict[limb] = package_handler_raw(args)
            flagDict[limb] = True

        if flagDict == toggleFlagDict:
            for x in range(len(flagDict) - 1):
                out.append(dataDict[orderDict[x]])
                    
            for x in flagDict:
                flagDict[x] = False
            
            rPacketReady = True
    
    
    #Assembles full packet for sending to Algorithm
    if rPacketReady:
        rPacketReady = False
        
        passToAlgorithm["t_raw"] = dataDict["rThigh"]
        passToAlgorithm["s_raw"] = dataDict["rShank"]
        passToAlgorithm["h_raw"] = dataDict["rHeel"]
        passToAlgorithm["b_raw"] = dataDict["lowBack"]
        
        packetReady = True
        
    #-------------------------------------------------------------#    
    #Code is broken into reader above and algorithm below for increased customization and ease of changing algorithm.
        
    
    if packetReady:
        packetReady = False
        
        #Update data
        t_raw = passToAlgorithm['t_raw']
        s_raw = passToAlgorithm['s_raw']
        h_raw = passToAlgorithm['h_raw']
        b_raw = passToAlgorithm['b_raw']
        
        gaitDetectThigh.gyX = t_raw[0]
        gaitDetectThigh.gyY = t_raw[1]
        gaitDetectThigh.gyZ = t_raw[2]
                  
        gaitDetectThigh.acX = t_raw[3]
        gaitDetectThigh.acY = t_raw[4]
        gaitDetectThigh.acZ = t_raw[5]
        
        gaitDetectShank.gyX = s_raw[0]
        gaitDetectShank.gyY = s_raw[1]
        gaitDetectShank.gyZ = s_raw[2]
                  
        gaitDetectShank.acX = s_raw[3]
        gaitDetectShank.acY = s_raw[4]
        gaitDetectShank.acZ = s_raw[5]
        
        gaitDetectHeel.gyX = h_raw[0]
        gaitDetectHeel.gyY = s_raw[1]
        gaitDetectHeel.gyZ = s_raw[2]
        
        gaitDetectHeel.acX = s_raw[3]
        gaitDetectHeel.acY = s_raw[4]
        gaitDetectHeel.acZ = s_raw[5]
        
        forward_acceleration = b_raw[5]
        
        
        gaitDetectShank.testVal()
        
        gaitDetectThigh.angleCalc(gaitDetectShank)
        gaitDetectShank.angleCalc(gaitDetectShank)
        gaitDetectHeel.angleCalc(gaitDetectShank)
        
        gaitDetectThigh.angularAcceleration()
        gaitDetectShank.angularAcceleration()
	
        dd_q_t = gaitDetectThigh.angularAcceleration
        dd_q_s = gaitDetectShank.angularAcceleration

        outputString = f"{timeToRun} {gaitDetectShank.gaitStage} {gaitDetectThigh.zAngle} {gaitDetectThigh.gyZ} {gaitDetectShank.zAngle} {gaitDetectShank.gyZ} {gaitDetectHeel.zAngle} {gaitDetectHeel.gyZ}"
        print(outputString)
        fileDump.write(outputString)
        
        if slipAlgorithm(gaitDetectShank.zAngle, gaitDetectThigh.zAngle, gaitDetectShank.gyZ, gaitDetectThigh.gyZ, dd_q_s, dd_q_t, gaitDetectThigh.acY, gaitDetectHeel.acX):
            ardno("ac")
            print("activated")
            fileDump.write("SLIP DETECTED\n")

        
        


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
    dispatcher.map("/Chordata/r*", data_handler)
    dispatcher.set_default_handler(default_handler)
    
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()  # Blocks forever



if __name__ == "__main__":
    bodyParameters(mass, height)
    gaitDetectThigh = gaitDetect()
    gaitDetectShank = gaitDetect()
    gaitDetectHeel = gaitDetect()
    
    fileDump = open("algDump.txt", "w+")
    fileDump.write("Thigh, Shank, Heel - gyX, gyY, gyZ, acX, acY, acZ, zAngle\n")
    
    main_func(ip, port)
    client.close()
