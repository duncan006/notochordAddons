#!/usr/bin/env python3
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from scipy.spatial.transform import Rotation
import math
import time

global testBool, varType, pullTime, timeStart
global unwrapX0, unwrapY0, unwrapZ0, unwrapX1, unwrapY1, unwrapZ1
global eBool0, rBool0, eBool1, rBool1, outR0, outE0, outR1, outE1
testBool = True
timeStart = time.time()
rBool0 = False
eBool0 = False
rBool1 = False
eBool1 = False

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
    global testBool, varType, pullTime, timeStart
    global unwrapX0, unwrapY0, unwrapZ0, unwrapX1, unwrapY1, unwrapZ1
    global eBool0, rBool0, eBool1, rBool1, outR0, outE0, outR1, outE1
    
    if testBool == True:
        testBool = False
        unwrapX0 = unwrapper(0)
        unwrapY0 = unwrapper(0)
        unwrapZ0 = unwrapper(0)
        unwrapX1 = unwrapper(0)
        unwrapY1 = unwrapper(0)
        unwrapZ1 = unwrapper(0)
    
    varType = address[10]
    
    if varType == "r":
        
        sensorNum = int(address[len(address) - 1])
        
        if sensorNum == 0:
            rBool0 = True
            outR0 = f"{varType}{sensorNum}"
            for pos, x in enumerate(args):
                if pos < 3:
                    outR0 += f" {x * .07}"
                elif pos >=3 and pos < 6:
                    outR0 += f" {x * .000244}"
                elif pos >= 6:
                    outR0 += f" {x * .00014}"
        elif sensorNum == 1:
            rBool1 = True
            outR1 = f"{varType}{sensorNum}"
            for pos, x in enumerate(args):
                if pos < 3:
                    outR0 += f" {x * .07}"
                elif pos >=3 and pos < 6:
                    outR0 += f" {x * .000244}"
                elif pos >= 6:
                    outR0 += f" {x * .00014}"
        
    elif varType == "q":
        x,y,z = package_handler_q(args)
        
        sensorNum = int(address[len(address) - 1])
        
        if sensorNum == 0:
            eBool0 = True
            x = unwrapX0.unwrap(x)
            y = unwrapY0.unwrap(y)
            z = unwrapZ0.unwrap(z)
            outE0 = f"{varType}{sensorNum} {x} {y} {z}"
        
        elif sensorNum == 1:
            eBool1 = True
            x = unwrapX1.unwrap(x)
            y = unwrapY1.unwrap(y)
            z = unwrapZ1.unwrap(z)
            outE1 = f"{varType}{sensorNum} {x} {y} {z}"
        
        
        
    if eBool1 and rBool1 and eBool0 and rBool0:
        eBool0 = False
        eBool1 = False
        rBool0 = False
        rBool1 = False
        
        try:
            #out = f"{time.time() - timeStart} {time.time() - pullTime} {outR0} {outE0} {outR1} {outE1}"
            out = f"{time.time() - timeStart} {time.time() - pullTime} {outR0} {outR1}"
        except NameError:
            pass
        
        pullTime = time.time()
        
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
