# from this video: 
# https://www.youtube.com/watch?v=-toNMaS4SeQ

import cv2
import mediapipe as mp
import numpy as np
import time
import socket
import sys

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

if bSocket:
	try:
		sendData = str(["reset", []])
		mysocket.send(sendData.encode())
	except:
		pass

if bSocket:
	close_socket(mysocket)