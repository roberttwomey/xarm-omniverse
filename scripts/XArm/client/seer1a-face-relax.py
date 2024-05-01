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


mp_face_mesh = mp.solutions.face_mesh

face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)

mp_drawing = mp.solutions.drawing_utils

drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture(1)

cv2.namedWindow('Head Pose Estimation', flags=cv2.WINDOW_GUI_NORMAL)

last_face = 0

count = 0
try: 
	while cap.isOpened():
		success, image = cap.read()

		start = time.time()

		# upside down
		# image = cv2.cvtColor(cv2.flip(image, -1), cv2.COLOR_BGR2RGB)

		# razer kyo pro - right side up
		image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)

		# improve performance
		image.flags.writeable = False

		# get the results
		results = face_mesh.process(image)

		# improve performance
		image.flags.writeable = True

		# convert colorspace
		image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)


		img_h, img_w, img_c = image.shape
		face_3d = []
		face_2d = []
		nose_norm = []
		

		if results.multi_face_landmarks:
			for face_landmarks in results.multi_face_landmarks:
				for idx, lm in enumerate(face_landmarks.landmark):
					if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
						if idx == 1:
							nose_2d = (lm.x * img_w, lm.y * img_h)
							nose_norm = (lm.x, lm.y, lm.z)
							nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 3000)

						x, y = int(lm.x * img_w), int(lm.y * img_h)
				
						cv2.circle(image, (x, y), 10, (0, 0, 255), -1)

						face_2d.append([x, y])

						face_3d.append([x, y, lm.z])

				# convert to numpy array
				face_2d = np.array(face_2d, dtype=np.float64)

				# convert to np array
				face_3d = np.array(face_3d, dtype=np.float64)

				# the camera matrix focal length
				focal_length = 1 * img_w

				cam_matrix = np.array( [
					[focal_length, 0, img_h/2],
					[0, focal_length, img_w/2],
					[0, 0, 1]
					])

				# the distortion paramaters
				dist_matrix = np.zeros((4, 1), dtype=np.float64)

				# solve PnP
				success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)

				# print("rot vec: ", rot_vec, "\ntrans vec: ", trans_vec)
				#print("nose_3d", nose_3d)

				# get rotatinal matrix
				rmat, jac = cv2.Rodrigues(rot_vec)

				# sget angles
				angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)

				# print("Q: ", Qx, Qy, Qz)

				# get the y rotation degree
				x_rot = angles[0] * 360
				y_rot = angles[1] * 360
				z_rot = angles[2] * 360

				# see where the user's head is tilting
				if y_rot < -10:
					text = "Looking Left"
				elif y_rot > 10:
					text = "Looking Right"
				elif x_rot < -10:
					text = "Looking Down"
				elif x_rot > 10:
					text = "Looking Up"
				else: 
					text = "Looking Forward"

				# display the nose direction
				nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rot_vec, trans_vec, cam_matrix, dist_matrix)
				# print("nose 3d proj:", nose_3d_projection)
				# print("Xr Yr Zr", x_rot, y_rot, z_rot)

				# compute world xyz pose of object
				# https://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point


				p1 = (int(nose_2d[0]), int(nose_2d[1]))
				p2 = (int(nose_2d[0] + y_rot * 10), int(nose_2d[1] - x_rot * 10))

				cv2.line(image, p1, p2, (255, 0, 0), 3)
				
				# add the text on the image
				# scale = 0.01 # this value can be from 0 to 1 (0,1] to change the size of the text relative to the image
				# fontScale = min(img_w, img_h)/(25/scale)
				fontScale = 0.8
				cv2.putText(image, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 2)
				cv2.putText(image, "x_rot: "+str(np.round(x_rot, 2)), (400, 50), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 2)
				cv2.putText(image, "y_rot: "+str(np.round(y_rot, 2)), (400, 100), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 2)
				cv2.putText(image, "z_rot: "+str(np.round(z_rot, 2)), (400, 150), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 2)
				xdiff = nose_norm[0]-0.5
				ydiff = nose_norm[1]-0.5
				# zdist = 0.030 - ((400.0 + nose_3d[2])/10000.0) # arbitrary offset
				# zdiff = 4*(15*nose_3d[2]/10000.0 + 0.01 + 0.35) # simple-face
				zdiff = 4*(15*nose_3d[2]/10000.0 + 0.01 + 0.35)-0.7 # INCOSE IW 2024
				# if (zdiff < 0):
					# print(zdiff)
					# zdiff = 0
				cv2.putText(image, "xdiff: "+str(np.round(xdiff, 3)), (400, 200), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 2)
				cv2.putText(image, "ydiff: "+str(np.round(ydiff, 3)), (400, 250), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 2)
				cv2.putText(image, "zdiff: "+str(np.round(zdiff, 3)), (400, 300), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 2)

			end = time.time()
			totalTime = end-start
			last_face = end

			fps = 1/totalTime
			# print("FPS: ", fps)

			cv2.putText(image, f'FPS: {int(fps)}', (20, 450), cv2.FONT_HERSHEY_SIMPLEX, fontScale, (0, 255, 0), 2)

			mp_drawing.draw_landmarks(
				image=image,
				landmark_list=face_landmarks,
				connections=mp_face_mesh.FACEMESH_CONTOURS,
				landmark_drawing_spec = drawing_spec,
				connection_drawing_spec=drawing_spec)

			if bSocket:
				try:
					count += 1
					if count % 3 == 0:
						print("sending:", ["face", [x_rot, y_rot*1.0, z_rot, xdiff, ydiff, zdiff]])
						count = 0
					# sendData = str([x_rot, y_rot*-1.0, z_rot, xdiff, ydiff, zdiff])
					sendData = str(["face", [x_rot, y_rot*-1.0, z_rot, xdiff, ydiff, zdiff]])
					mysocket.send(sendData.encode())
				except:
					pass
		else:
			if time.time() - last_face > 2.0:
				sendData = str(["relax", []])
				mysocket.send(sendData.encode())
				time.sleep(0.1)
		cv2.imshow('Head Pose Estimation', image)

		if cv2.waitKey(5) & 0xFF == 27:
			break
except KeyboardInterrupt:
	print("quitting")

if bSocket:
	close_socket(mysocket)

cap.release()


