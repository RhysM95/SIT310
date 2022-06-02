#!/usr/bin/python3
import time

import Tello
if __name__ == "__main__":    
    tello = Tello.Controller()
    tello.send_command("command")
    time.sleep(2)
    tello.send_command("takeoff")
    time.sleep(10)
    tello.send_command("cw 360")
    time.sleep(7)
    tello.send_command("land")
    time.sleep(3)
    
