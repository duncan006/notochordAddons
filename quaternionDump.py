#!/usr/bin/env python3

#TO CUSTOMIZE:
#Lines 14-17 can be changed to change the format of the data
#Line 30 can be changed to satisfy how your client receives data

from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer


def data_handler(address, *args):
    
    #Replace these 4 lines with whatever format you need
    dataString = ''
    dataString += str(address)
    dataString += ":"
    dataString += str(args)
    
    send_package(dataString)


def default_handler(address, *args):
    out = 'DEFAULT '
    out += str(address)
    out += str(args)



def send_package(out):
    print(out) #<--------------Change this function to be program specific



def main_func():
    dispatcher = Dispatcher()
    dispatcher.map("/%/*", data_handler)
    dispatcher.set_default_handler(default_handler)
    
    ip = "localhost"
    port = 6565
    
    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()  # Blocks forever



if __name__ == "__main__":
    main_func()
