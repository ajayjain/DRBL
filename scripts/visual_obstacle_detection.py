#!/usr/bin/python

import cv2, cv
import numpy as np

cap = cv2.VideoCapture(0)

while True:
	ret, frame = cap.read()
	frame = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
	edges = cv2.Canny(frame, 300, 400)
	cv2.imshow('camera', frame)
	cv2.imshow('edges', edges)

	for c in xrange(edges.shape[1]):
		col = edges[:,c]
		col[col.argmax():].fill(255)

	cv2.imshow('side fill', edges)

	kernel = np.ones((5,5),np.uint8)
	erosion = cv2.dilate(edges,kernel,iterations = 1)
	cv2.imshow('erode', erosion)

	print erosion.shape, frame.shape
	added = cv2.add(erosion, frame)
	cv2.imshow('added', added)

	# dst = cv2.cornerHarris(erosion,2,3,0.04)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()