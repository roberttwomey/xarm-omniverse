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
time_ascend = 3.0*60.0
time_descend = 5.0*60.0

count = 0

try: 
	while True:
		try:
			if bSocket:
				try:
					time_elapsed = time.time() - starttime
					# sendData = str(["pos", [
					# 	radius*math.sin(time_elapsed % (2*math.pi)), radius*math.cos(time_elapsed % (2*math.pi)), 0.34, 
					# 	0., 0., 0.]])
					
					if time_elapsed < time_ascend:
						zpos = 0.3 + (0.7*time_elapsed/time_ascend)
						if time_elapsed < 0.4 * time_ascend:
							radius = 0.3 + (0.3*time_elapsed/(0.4*time_ascend))
						else:
							radius = 0.3 + (0.3*(time_ascend-time_elapsed)/(0.6*time_ascend))
					else:
						zpos = 0.3 + (0.7*(time_descend - time_elapsed)/120.0)
						
					# zpos = 0.55 + 0.25 * math.sin((0.05*time_elapsed % (2*math.pi))-math.pi)
					
					xpos = radius*math.sin((0.1*time_elapsed % (2*math.pi))-math.pi)
					ypos = radius*math.cos((0.1*time_elapsed % (2*math.pi))-math.pi)
					
					sendData = str(["pos", [
						xpos, ypos, zpos, 
						0., 0., 0.]])
					
					# count+=1
					# if count % 10 == 0:
					# 	print(f"{time_elapsed} sec: {sendData}")

					print(f"{time_elapsed} sec: {sendData}")

					if first_pos:
						thisjuk = input("type to start")
						first_pos = False

					mysocket.send(sendData.encode())
				except:
					pass
				
				time.sleep(0.1)
				if time_elapsed > time_descend:
					starttime = time.time()
		except KeyboardInterrupt:
			print("time ", time_elapsed)
			input("pause. enter to continue")
			starttime = time.time() - time_elapsed
			print("new time ", time.time()-starttime)
except KeyboardInterrupt:
	print("quitting")

# relax back to home
# for _ in range(100):
# 	sendData = str(["relax", []])
# 	mysocket.send(sendData.encode())
# 	time.sleep(0.1)
# print("done.")

if bSocket:
	close_socket(mysocket)