import cv2
import numpy as np
import pdb
import os
#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template=None):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	########## YOUR CODE STARTS HERE ##########
	img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) 	#convert the BGR image to HSV colour space

	# For HSV, Hue range is [0,179], Saturation range is [0,255] and Value range is [0,255]
	#create a mask for orange colour using inRange function
	# mask = cv2.inRange(img_hsv, np.array([1,90,60]), np.array([40,255,255])) # range for city in stata
	mask = cv2.inRange(img_hsv, np.array([1,100,80]), np.array([40,255,255])) # range for city in stata

	# mask = cv2.inRange(img_hsv, np.array([10,130,100]), np.array([45,255,255])) # range for city in johnson
    

	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
	
	contourAreas = [cv2.contourArea(c) for c in contours] # area of contours
	cone_cnt = contours[contourAreas.index(max(contourAreas))]# the biggest match is most likely

	x, y, w, h = cv2.boundingRect(cone_cnt)
	bounding_box = ((x,y),(x+w,y+h))

	if False:
		cv2.drawContours(img, contours, -1, (0,255,0), 3)
		cv2.rectangle(img, bounding_box[0], bounding_box[1], (0,0, 255), 2)
		# cv2.imshow("mask", mask)
		cv2.imshow("contour", img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

	return bounding_box

# test imgs
# img = cv2.imread("scripts/computer_vision/cityline.png")

# # slicing image
# lower = 0
# upper = 1

# img[:int(lower*img.shape[0])] = (0,0,0)
# img[int(upper*img.shape[0]):] = (0,0,0)

# cv2.imshow("slice", img)


# cv2.waitKey(0)
# cv2.destroyAllWindows()

# cd_color_segmentation(img)
