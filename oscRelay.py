#!/usr/bin/env python3

from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from scipy.spatial.transform import Rotation as R
import socket
import json
import numpy as np
global client, varType, dataDict, flagDict, qDataDict, qFlagDict, toggleFlagDict

#User Input
#True toggle flags will be included in payload. False toggle flags will not be included.
#If a toggle flag is set to true, and the sensor is not, the payload will not send.
#TODO: Get sensors from XML file

toggleFlagDict = {
	"rThigh": True,
	"rShank": False,
	"rHeel": False,
	"lowBack": False,
	"lThigh": False,
	"lShank": False,
	"lHeel": False,
	"rArm": False,
	"rForarm": False,
	"rHand": False,
	"topBack": False,
	"Head": False,
	"lArm": False,
	"lForarm": False,
	"lHand": False,
}

#Variable Initializations

dataDict = {
	"rThigh":  [],
	"rShank":  [],
	"rHeel":  [],
	"lowBack":  [],
	"lThigh":  [],
	"lShank":  [],
	"lHeel":  [],
	"rArm":  [],
	"rForarm":  [],
	"rHand":  [],
	"topBack":  [],
	"Head":  [],
	"lArm":  [],
	"lForarm":  [],
	"lHand":  [],
}

qDataDict = {
	"rThigh":  [],
	"rShank":  [],
	"rHeel":  [],
	"lowBack":  [],
	"lThigh":  [],
	"lShank":  [],
	"lHeel":  [],
	"rArm":  [],
	"rForarm":  [],
	"rHand":  [],
	"topBack":  [],
	"Head":  [],
	"lArm":  [],
	"lForarm":  [],
	"lHand":  [],
}

flagDict = {
	"rThigh": False,
	"rShank": False,
	"rHeel": False,
	"lowBack": False,
	"lThigh": False,
	"lShank": False,
	"lHeel": False,
	"rArm": False,
	"rForarm": False,
	"rHand": False,
	"topBack": False,
	"Head": False,
	"lArm": False,
	"lForarm": False,
	"lHand": False,
}

qFlagDict = {
	"rThigh": False,
	"rShank": False,
	"rHeel": False,
	"lowBack": False,
	"lThigh": False,
	"lShank": False,
	"lHeel": False,
	"rArm": False,
	"rForarm": False,
	"rHand": False,
	"topBack": False,
	"Head": False,
	"lArm": False,
	"lForarm": False,
	"lHand": False,
}

addressDict = {
	"10": "rThigh",
	"11": "rShank",
	"12": "rHeel",
	"20": "lowBack",
	"30": "lThigh",
	"31": "lShank",
	"32": "lHeel",
	"40": "rArm",
	"41": "rForarm",
	"42": "rHand",
	"50": "topBack",
	"51": "Head",
	"60": "lArm",
	"61": "lForarm",
	"62": "lHand"
}

orderDict = {
	0: "rThigh",
	1: "rShank",
	2: "rHeel",
	3: "lowBack",
	4: "lThigh",
	5: "lShank",
	6: "lHeel",
	7: "rArm",
	8: "rForarm",
	9: "rHand",
    10: "topBack",
	11: "Head",
	12: "lArm",
	13: "lForarm",
	14: "lHand"
}


def package_handler_raw(tup):
    #Simplified package for deciding what raw measurements will be in your data    
    keep_gys_x = False
    keep_gys_y = False
    keep_gys_z = True
    keep_acc_x = True
    keep_acc_y = True
    keep_acc_z = True
    keep_mag_x = False
    keep_mag_y = False
    keep_mag_z = False
    
    out = []
    
    if keep_gys_x == True: out.append(tup[0])    
    if keep_gys_y == True: out.append(tup[1])    
    if keep_gys_z == True: out.append(tup[2])    
    if keep_acc_x == True: out.append(tup[3])     
    if keep_acc_y == True: out.append(tup[4])     
    if keep_acc_z == True: out.append(tup[5])    
    if keep_mag_x == True: out.append(tup[6])   
    if keep_mag_y == True: out.append(tup[7])   
    if keep_mag_z == True: out.append(tup[8])
    
    return out



def package_handler_q(tup):
    keepX = True
    keepY = False
    keepZ = True
    
    out = []
    w = tup[0]
    x = tup[1]
    y = tup[2]
    z = tup[3]
    
    r = R.from_quat([x,y,z,w])
    e = r.as_euler('xyz', degrees=False)
    
    if keepX == True:
        out.append(e[0])
    if keepY == True:
        out.append(e[1])
    if keepZ == True:
        out.append(e[2])
    
    return out


  
def address_handler(address, *args):
    global varType
    varType = address[13]



def data_handler(address, *args):
    global varType, dataDict, flagDict, qDataDict, qFlagDict, toggleFlagDict
    addr = ''
    addr += str(address[9])
    addr += str(address[11])
    limb = addressDict[addr]
    out = []
    
    if varType == "q":
        qDataDict[limb] = package_handler_q(args)
        qFlagDict[limb] = True
    if varType == "r":
        dataDict[limb] = package_handler_raw(args)
        flagDict[limb] = True
    
    if qFlagDict == toggleFlagDict:
        out.append("e")
        for x in range(15):
            if toggleFlagDict[orderDict[x]] == True:
                out.append(qDataDict[orderDict[x]])
                
        for x in qFlagDict:
            qFlagDict[x] = False
        
        send_package(out)
    
    if flagDict == toggleFlagDict:
        out.append("r")
        for x in range(15):
            if toggleFlagDict[orderDict[x]] == True:
                out.append(dataDict[orderDict[x]])
                
        for x in flagDict:
            flagDict[x] = False
        
        send_package(out)



def default_handler(address, *args):
    out = 'DEFAULT '
    out += str(address)
    out += str(args)



def send_package(out):
    global client
    transmit = json.dumps(out)
    client.send(transmit.encode())



def main_func():
    global client
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(('localhost', 30001))
    
    dispatcher = Dispatcher()
    dispatcher.map("/%%/*", address_handler)
    dispatcher.map("/%/*", data_handler)
    dispatcher.set_default_handler(default_handler)
    
    ip = "localhost"
    port = 6565
    
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()  # Blocks forever



if __name__ == "__main__":
    main_func()
    client.close()
