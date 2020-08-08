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
global gaitDetectRHeel, gaitDetectRShank, gaitDetectRThigh
global gaitDetectLHeel, gaitDetectLShank, gaitDetectLThigh
global objects



ip = "localhost"
port = 6565

mass = 60 #kg
height = 180 #cm

timeCurrent = time.time()


toggleFlagDict = {
	"rThigh": True,
	"rShank": True,
	"rHeel": True,
	"lThigh": True,
	"lShank": True,
	"lHeel": True,
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
	"lThigh": [],
	"lShank": [],
	"lHeel": [],
	"lowBack":  [],
}

flagDict = {
	"rThigh": False,
	"rShank": False,
	"rHeel": False,
	"lThigh": False,
	"lShank": False,
	"lHeel": False,
    "lowBack": False,
}

addressDict = {
	"10": "rThigh",
	"11": "rShank",
	"12": "rHeel",
	"30": "lThigh",
	"31": "lShank",
	"32": "lHeel",
	"20": "lowBack",
}

orderDict = {
	0: "rThigh",
	1: "rShank",
	2: "rHeel",
	3: "lThigh",
	4: "lShank",
	5: "lHeel",
	6: "lowBack",
}

passToAlgorithm = {
	"rt_raw": [],
	"rs_raw": [],
	"rh_raw": [],
	"lt_raw": [],
	"ls_raw": [],
	"lh_raw": [],
	"b_raw": [],
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
        
        self.mgX = 0
        self.mgY = 0
        self.mgZ = 0

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
    
    def angularAccelerationCalc(self):
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
                
        elif gaitDetectObject.standing == True:
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
    
    for pos, x in enumerate(tup):
        if pos < 3:
            out.append(x * .07)
        elif pos >=3 and pos < 6:
            out.append(x * .000244 * 9.81)
        elif pos >= 6:
            out.append(x * .00014)
            
    return out

	

def slipAlgorithm(pelvisAcc, forwardFootAcc, L_hh):
    global fileDump

    slip_constant = 2.83 #or 1.87
    beta = 2.718
    gamma = -562 #or -377 #deg/s	
    
    dd_q_hh = (pelvisAcc - forwardFootAcc) / L_hh
    
    slip_indicator = forwardFootAcc / (beta ** (dd_q_hh - gamma))
    
    fileDump.write(f"{slip_indicator}\t")
	
    if slip_indicator >= slip_constant:
        return True
    else:
        return False



def data_handler(address, *args):
    global timeStart, timeCurrent, varType
    global dataDict, flagDict, qDataDict, qFlagDict, toggleFlagDict
    global packetReady, rPacketReady, passToAlgorithm
    global winters, fileDump
    global gaitDetectRHeel, gaitDetectRShank, gaitDetectRThigh
    global gaitDetectLHeel, gaitDetectLShank, gaitDetectLThigh
    global objects
    
    
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
        
        passToAlgorithm["rt_raw"] = dataDict["rThigh"]
        passToAlgorithm["rs_raw"] = dataDict["rShank"]
        passToAlgorithm["rh_raw"] = dataDict["rHeel"]
        passToAlgorithm["lt_raw"] = dataDict["lThigh"]
        passToAlgorithm["ls_raw"] = dataDict["lShank"]
        passToAlgorithm["lh_raw"] = dataDict["lHeel"]
        passToAlgorithm["b_raw"] = dataDict["lowBack"]
        
        packetReady = True
        
    #-------------------------------------------------------------#    
    #Code is broken into reader above and algorithm below for increased customization and ease of changing algorithm.
        
    
    if packetReady:
        packetReady = False
		
		timeLastRun = timeCurrent
		timeCurrent = time.time()
		timeToRun = timeCurrent - timeLastRun
        
        #Update data
        rt_raw = passToAlgorithm['rt_raw']
        rs_raw = passToAlgorithm['rs_raw']
        rh_raw = passToAlgorithm['rh_raw']
        lt_raw = passToAlgorithm['lt_raw']
        ls_raw = passToAlgorithm['ls_raw']
        lh_raw = passToAlgorithm['lh_raw']
        b_raw = passToAlgorithm['b_raw']
        
        #Values are moved around to equalize the axes of the IMUs.
        #Does not put IMUs on a global coordinate system.
        #Only sets local axes to be the same.
		
        #Right Thigh - no flipped values
        gaitDetectRThigh.gyX = rt_raw[0]
        gaitDetectRThigh.gyY = rt_raw[1]
        gaitDetectRThigh.gyZ = rt_raw[2]
                  
        gaitDetectRThigh.acX = rt_raw[3]
        gaitDetectRThigh.acY = rt_raw[4]
        gaitDetectRThigh.acZ = rt_raw[5]
                  
        gaitDetectRThigh.mgX = rt_raw[6]
        gaitDetectRThigh.mgY = rt_raw[7]
        gaitDetectRThigh.mgZ = rt_raw[8]
        
        #Right Shank - no flipped values
        gaitDetectRShank.gyX = rs_raw[0]
        gaitDetectRShank.gyY = rs_raw[1]
        gaitDetectRShank.gyZ = rs_raw[2]
                  
        gaitDetectRShank.acX = rs_raw[3]
        gaitDetectRShank.acY = rs_raw[4]
        gaitDetectRShank.acZ = rs_raw[5]
                  
        gaitDetectRShank.mgX = rs_raw[6]
        gaitDetectRShank.mgY = rs_raw[7]
        gaitDetectRShank.mgZ = rs_raw[8]
        
        #Right Heel - X and Y axes flipped, X negated
        gaitDetectRHeel.gyX = -rh_raw[1]
        gaitDetectRHeel.gyY = rh_raw[0]
        gaitDetectRHeel.gyZ = rh_raw[2]
        
        gaitDetectRHeel.acX = -rh_raw[4]
        gaitDetectRHeel.acY = rh_raw[3]
        gaitDetectRHeel.acZ = rh_raw[5]
        
        gaitDetectRHeel.mgX = -rh_raw[7]
        gaitDetectRHeel.mgY = rh_raw[6]
        gaitDetectRHeel.mgZ = rh_raw[8]
		
        #Left Thigh - X and Z values negated
        gaitDetectLThigh.gyX = -lt_raw[0]
        gaitDetectLThigh.gyY =  lt_raw[1]
        gaitDetectLThigh.gyZ = -lt_raw[2]
                               
        gaitDetectLThigh.acX = -lt_raw[3]
        gaitDetectLThigh.acY =  lt_raw[4]
        gaitDetectLThigh.acZ = -lt_raw[5]
                               
        gaitDetectLThigh.mgX = -lt_raw[6]
        gaitDetectLThigh.mgY =  lt_raw[7]
        gaitDetectLThigh.mgZ = -lt_raw[8]
        
        #Left Shank - X and Z values negated		
        gaitDetectLShank.gyX = -ls_raw[0]
        gaitDetectLShank.gyY =  ls_raw[1]
        gaitDetectLShank.gyZ = -ls_raw[2]
                               
        gaitDetectLShank.acX = -ls_raw[3]
        gaitDetectLShank.acY =  ls_raw[4]
        gaitDetectLShank.acZ = -ls_raw[5]
                               
        gaitDetectLShank.mgX = -ls_raw[6]
        gaitDetectLShank.mgY =  ls_raw[7]
        gaitDetectLShank.mgZ = -ls_raw[8]
                  
        #Left Heel - X and Y axes flipped, then X, Y, and Z values negated.
        gaitDetectLHeel.gyX = -lh_raw[1]
        gaitDetectLHeel.gyY =  lh_raw[0]
        gaitDetectLHeel.gyZ = -lh_raw[2]
                              
        gaitDetectLHeel.acX = -lh_raw[4]
        gaitDetectLHeel.acY =  lh_raw[3]
        gaitDetectLHeel.acZ = -lh_raw[5]
                              
        gaitDetectLHeel.mgX = -lh_raw[7]
        gaitDetectLHeel.mgY =  lh_raw[6]
        gaitDetectLHeel.mgZ = -lh_raw[8]
        
        #Lower Back - X and Z values flipped, X value negated
        gaitDetectLowBack.gyX = -b_raw[2]
        gaitDetectLowBack.gyY =  b_raw[1]
        gaitDetectLowBack.gyZ =  b_raw[0]
                       
        gaitDetectLowBack.acX = -b_raw[5]
        gaitDetectLowBack.acY =  b_raw[4]
        gaitDetectLowBack.acZ =  b_raw[3]
                   
        gaitDetectLowBack.mgX = -b_raw[8]
        gaitDetectLowBack.mgY =  b_raw[7]
        gaitDetectLowBack.mgZ =  b_raw[6]
        
        
        gaitDetectRShank.testVal()
        gaitDetectLShank.testVal()
		
        
        gaitDetectRThigh.angleCalc(gaitDetectRShank)
        gaitDetectRShank.angleCalc(gaitDetectRShank)
        gaitDetectRHeel.angleCalc(gaitDetectRShank)
		
        gaitDetectLThigh.angleCalc(gaitDetectLShank)
        gaitDetectLShank.angleCalc(gaitDetectLShank)
        gaitDetectLHeel.angleCalc(gaitDetectLShank)
        
		
        #gaitDetectRThigh.angularAccelerationCalc()
        #gaitDetectRShank.angularAccelerationCalc()

        #gaitDetectLThigh.angularAccelerationCalc()
        #gaitDetectLShank.angularAccelerationCalc()

		
        outputString = f"{timeToRun}\t{gaitDetectRShank.gaitStage}\t{gaitDetectLShank.gaitStage}\t\t"
		
        for x in objects:
            outputString += f"{x.gyX}\t"
            outputString += f"{x.gyY}\t"
            outputString += f"{x.gyZ}\t\t"

            outputString += f"{x.acX}\t"
            outputString += f"{x.acY}\t"
            outputString += f"{x.acZ}\t\t"

            outputString += f"{x.mgX}\t"
            outputString += f"{x.mgY}\t"
            outputString += f"{x.mgZ}\t\t\t"

        for x in objects:
            outputString += f"{x.zAngle}\t"
		
        print(outputString)
        fileDump.write(f"{outputString}")
        
		
        if slipAlgorithm(gaitDetectLowBack.acX, gaitDetectRHeel.acX, 1):
            #ardno("ac")
            #print("activated")
            fileDump.write(" RIGHT_SLIP_DETECTED ")
        if slipAlgorithm(gaitDetectLowBack.acX, gaitDetectLHeel.acX, 1):
            fileDump.write(" LEFT_SLIP_DETECTED ")

        
        fileDump.write("\n")


def default_handler(address, *args):
    out = 'DEFAULT '
    out += str(address)
    out += str(args)
    #print(out)



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
    
    gaitDetectRThigh = gaitDetect()
    gaitDetectRShank = gaitDetect()
    gaitDetectRHeel = gaitDetect()
	
    gaitDetectLThigh = gaitDetect()
    gaitDetectLShank = gaitDetect()
    gaitDetectLHeel = gaitDetect()
	
    gaitDetectLowBack = gaitDetect()
	
    objects = [gaitDetectRThigh, gaitDetectRShank, gaitDetectRHeel, gaitDetectLThigh, gaitDetectLShank, gaitDetectLHeel, gaitDetectLowBack]
    stringObjects = ["RThigh", "RShank", "RHeel", "LThigh", "LShank", "LHeel", "LowBack"]
    stringAxes = ["x","y","z"]
    stringSensors = ["gy","ac","mg"]
	
	
    fileDump = open("algDump.txt", "w+")
    header = "timeToRun\tgaitStageR\tgaitStageL\t\t"
    for x in stringObjects:
        for y in stringSensors:
            for z in stringAxes:
                header += f"{y}/{z}/{x}\t"
            header += f"\t"
        header += f"\t"
	
    for x in stringObjects:
        header += f"zAngle{x}\t"
		
    header += f"slipRight\tslipLeft"
	
    header += f"\n"
    fileDump.write(header)
    
    main_func(ip, port)
    client.close()
