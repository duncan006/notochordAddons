#!/usr/bin/env python3

from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer

def default_handler(address, *args):
    print(f"{address}: {args}")
        
def main_func():
    ip = "localhost"
    port = 6565
    
    dispatcher = Dispatcher()
    dispatcher.set_default_handler(default_handler)

    server = BlockingOSCUDPServer((ip, port), dispatcher)
    server.serve_forever()

if __name__ == "__main__":
    main_func()
