#!/usr/bin/env python

# /media/ajay/5056EE7956EE5EEA/Users/Ajay/Desktop/gopro/DCIM/100GOPRO/repaired

from __future__ import print_function
from SimpleCV import Image, Color, VirtualCamera, Display

video = VirtualCamera('/media/ajay/5056EE7956EE5EEA/Users/Ajay/Desktop/gopro/DCIM/100GOPRO/repaired/GOPR0429.MP4', 'video')
display = Display()

while display.isNotDone():
	img = video.getImage()
	try:
		dist = img - img.colorDistance(Color.BLUE)
		dist.show()
	except KeyboardInterrupt:
		display.done = True
	if display.mouseRight:
		display.done = True
display.quit()