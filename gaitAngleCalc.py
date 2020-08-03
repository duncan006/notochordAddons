#!/usr/bin/env python3
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from scipy.spatial.transform import Rotation
import math
import time

global testBool, unwrapX, unwrapY, unwrapZ, pullTime, timeStart, filtAcX, filtAcY, filtAcZ, filtGyX, filtGyY, filtGyZ, eBool, rBool, ePacket, rPacket, gaitDetectRight, zAngle, zCalib, zAngleChangeArr
testBool = True
timeStart = time.time()
rBool = False
eBool = False
wasStanding = False
rPacket = ""
ePacket = ""
zAngle = 0
zCalib = 0


class gaitDetect:
    def __init__(self, firstVar):
        self.firstVar = firstVar
        self.movingArr = [firstVar]
        self.significance = 0
        self.movingAvgAccuracy = 2
        self.movingAvg = 0
        self.lastAvg = 0
        self.timeLastHeelStrike = 0
        self.timeLastToeOff = 0
        self.gaitStage = 0 #1 for swing, 0 for stance
        self.eventTimer = .1
        self.standing = False
        self.standingLimit = 200
        self.concurrentZeroes = 0
        self.concurrentZeroesLimit = 5
        
    def testVal(self, nextVal):
        self.movingArr.append(nextVal)
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
        
        print(f"{self.gaitStage}")

        self.lastAvg = self.movingAvg



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
    
    out.append(e[0] * 180 / math.pi) #x-axis euler
    out.append(e[1] * 180 / math.pi) #y-axis euler
    out.append(e[2] * 180 / math.pi) #z-axis euler
    
    return out
    

def default_handler(address, *args):
    print(f"{address}: {args}")

    

def data_handler(address, *args):
    global testBool, varType, unwrapX, unwrapY, unwrapZ, pullTime, timeStart
    global ePacket, rPacket, eBool, rBool, gaitDetectRight, zAngle, zCalib, zAngleChangeArr, wasStanding
    
    varType = address[10]
    
    if varType == "r":
        rBool = True
        rPacket = "raw "
        
        for pos, x in enumerate(args):
            if pos < 3:
                rPacket += f" {x * .07}"
            elif pos >=3 and pos < 6:
                rPacket += f" {x * .000244}"
            elif pos >= 6:
                rPacket += f" {x * .00014}"
                
        rPacket += " "
        gyz = args[2]
            
    elif varType == "q":
        eBool = True
        x,y,z = package_handler_q(args)
        
        if testBool == True:
            testBool = False
            unwrapX = unwrapper(x)
            unwrapY = unwrapper(y)
            unwrapZ = unwrapper(z)
        else:
            ePacket = f"wrappedXYZ {x} {y} {z} "
            x = unwrapX.unwrap(x)
            y = unwrapY.unwrap(y)
            z = unwrapZ.unwrap(z)
            ePacket += f"unwrappedXYZ {x} {y} {z} "
        
    if rBool and eBool:
        rBool = False
        eBool = False
        
        out = ""
        #out += str(time.time() - timeStart)
        #out += " "
        try:
            timeToRun = time.time() - pullTime
        except NameError:
            timeToRun = 0
            
        out += f"{timeToRun} "
        
        gaitDetectRight.testVal(gyz / .07)
        zAngleChange = gyz * timeToRun
        zAngleChangeArray.append(zAngleChange)
        
        if len(zAngleChangeArray) > 5:
            zAngleChangeArray.pop(0)
        
        if gaitDetectRight.standing == False:
            if wasStanding == True:
                wasStanding = False
                zAngle += np.sum(zAngleChangeArray)
            else:
                zAngle += zAngleChange
                
        elif gaitDetectRight.standing == True:
            if zAngle > 0:
                zAngle -= 1
            elif zAngle < 0:
                zAngle += 1
            wasStanding = True
        
        pullTime = time.time()
    
        out += rPacket
        out += ePacket
        out += zAngle
        print(out)
        fileData.write(out)
        fileData.write("\n")
        
def main_func():
    ip = "localhost"
    port = 6565
    
    dispatcher = Dispatcher()
    dispatcher.map("/Chordata/*", data_handler)
    dispatcher.set_default_handler(default_handler)

    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()
    
    
if __name__ == "__main__":
    fileData = open("FilteredDataDump.txt", "w+")
    gaitDetectRight = gaitDetect(0)
    main_func()
    fileData.close()
