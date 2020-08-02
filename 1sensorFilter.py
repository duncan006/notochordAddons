#!/usr/bin/env python3
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from scipy.spatial.transform import Rotation
import math
import time

global testBool, unwrapX, unwrapY, unwrapZ, pullTime, timeStart, filtAcX, filtAcY, filtAcZ, filtGyX, filtGyY, filtGyZ, eBool, rBool, ePacket, rPacket
testBool = True
timeStart = time.time()
rBool = False
eBool = False
rPacket = ""
ePacket = ""

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
    
    out.append(e[0]) #x-axis euler
    out.append(e[1]) #y-axis euler
    out.append(e[2]) #z-axis euler
    
    return out
    

def default_handler(address, *args):
    print(f"{address}: {args}")

    

def data_handler(address, *args):
    global testBool, varType, unwrapX, unwrapY, unwrapZ, pullTime, timeStart
    global ePacket, rPacket, eBool, rBool
    
    varType = address[10]
    
    if varType == "r":
        rBool = True
        rPacket = "raw "
        
        for x in args:
            rPacket += f"{x} "
            
        for pos, x in enumerate(args):
            if pos < 3:
                rPacket += f" {x * .07}"
            elif pos >=3 and pos < 6:
                rPacket += f" {x * .000244}"
            elif pos >= 6:
                rPacket += f" {x * .00014}"
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
            ePacket += f"unwrappedXYZ {x} {y} {z}"
        
    if rBool and eBool:
        rBool = False
        eBool = False
        
        out = ""
        #out += str(time.time() - timeStart)
        #out += " "
        try:
            out += str(time.time() - pullTime)
            out += " "
        except NameError:
            out += " 0 "
            #pass
        
        pullTime = time.time()
    
        out += rPacket
        out += ePacket
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
    main_func()
    fileData.close()
