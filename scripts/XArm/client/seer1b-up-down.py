# from this video: 
# https://www.youtube.com/watch?v=-toNMaS4SeQ

import cv2
import mediapipe as mp
import numpy as np
import time
import socket
import sys
import math

bSocket = True

if (len(sys.argv) > 1):
	print(sys.argv)
	if sys.argv[1] == "--no-socket":
		bSocket = False


if bSocket:
	# open socket to omniverse machine
	mysocket = socket.socket()
	# mysocket.connect(('192.168.4.5',12346)) # easybake
	mysocket.connect(('127.0.0.1',12346))


def close_socket(thissocket):
    try:
        thissocket.shutdown(socket.SHUT_RDWR)
        thissocket.close()
        thissocket = None
    except socket.error as e:
        pass
    print("socket is closed")

starttime = time.time()
radius = 0.5
first_pos = True
startpos = [0.39, 0, 0.34, 0, 90, 0]



try: 
	while True:
		if bSocket:
			try:
				if first_pos:
					_ = input("press enter to start...")
					first_pos = False

				time_elapsed = starttime - time.time()

				ypos = 0.39+0.1*math.sin((0.5*time_elapsed % (2*math.pi))-math.pi)
				xpos = 0.0
				zpos = 0.5+0.16*math.cos((0.5*time_elapsed % (2*math.pi))-math.pi)
				sendData = str(["pos", [
					xpos, ypos, zpos, 
					90., 0., 0.]])
				
				mysocket.send(sendData.encode())
			except:
				pass
			
			time.sleep(0.1)

except KeyboardInterrupt:
	print("quitting. relax to home...", end="")


for _ in range(100):
	sendData = str(["relax", []])
	mysocket.send(sendData.encode())
	time.sleep(0.1)
print("done.")

if bSocket:
	close_socket(mysocket)
