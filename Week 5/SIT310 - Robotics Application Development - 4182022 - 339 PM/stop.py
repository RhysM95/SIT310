#!/usr/bin/python3
import time

import Tello
if __name__ == "__main__":
    tello = Tello.Controller()
    tello.send_command("command”)
    time.sleep(1)
    tello.send_command("emergency”)


