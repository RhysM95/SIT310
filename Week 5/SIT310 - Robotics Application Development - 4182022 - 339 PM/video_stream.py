#!/usr/bin/python3
import threading
import socket

#first install opencv and python bindings, apt install python3-opencv
import cv2
import time

import Tello

class VideoTello:
    def __init__(self):
        self._running = True
        self.video = cv2.VideoCapture("udp://@0.0.0.0:11111")

        
    def terminate(self):
        self._running = False
        self.video.release()
        cv2.destroyAllWindows()

    def recv(self):
        """ Handler for Tello states message """
        while self._running:
            try:
                ret, frame = self.video.read()
                if ret:
                    # Resize frame
                    height, width, _ = frame.shape
                    new_h = int(height / 2)
                    new_w = int(width / 2)

                    # Resize for improved performance
                    new_frame = cv2.resize(frame, (new_w, new_h))

                    # Display the resulting frame
                    cv2.imshow('Tello', new_frame)
                # Wait for display image frame
                # cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.waitKey(1)
            except Exception as err:
                print(err)


#request the video
import Tello
if __name__ == "__main__":    
    tello = Tello.Controller()
    print("sending command")
    tello.send_command("command")
    time.sleep(3)
    print("sending streamon")
    tello.send_command("streamon")
    #time.sleep(3)
    
print("starting video thread")
t = VideoTello()
recvThread = threading.Thread(target=t.recv)
recvThread.start()
print("video thread started")

