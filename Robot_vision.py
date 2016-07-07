# Algorithm to detect sidewalk based on back projecting the rgb and hsv values.
import cv2
import numpy as np 

def sidewalk_detection(img):
	img_org = img.copy()
	img_untouch = img.copy()
	width = img.shape[1]
	height = img.shape[0]

	# Set the region immediately infront of the vehicle as the region of interest(primary assumption)
	a = int(height/2) - 130
	b = int(height/2) + 130
	c = int(width/2) + 100
	d = int(width/2) - 100
	roi = img_untouch[d:c,a:b,:]
	
	# Convert RGB to HSV region of interst 
	roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

	# Obtain desirable RGB and HS threshold values(robust to illumination changes)
	roi_r_mean = np.mean(roi[:,:,2].ravel())
	roi_g_mean = np.mean(roi[:,:,1].ravel())
	roi_b_mean = np.mean(roi[:,:,0].ravel())
	roi_h_mean = np.mean(roi_hsv[:,:,0].ravel())
	roi_s_mean = np.mean(roi_hsv[:,:,1].ravel())

	roi_r_thresh = int(roi_r_mean - 40) 
	roi_g_thresh = int(roi_g_mean - 30) 
	roi_b_thresh = int(np.min([roi_b_mean+60,255])) 
	roi_s_thresh = int(np.min([roi_s_mean+50,255])) 

	# Remove Noise and obtain HSV of original image
	img = cv2.medianBlur(img,7)
	img = cv2.GaussianBlur(img,(9,9),0)
	img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# Apply the threshold levels above to the whole image
	img_org[img_org[:,:,0] > roi_b_thresh] = 0
	img_org[img_hsv[:,:,1] > roi_s_thresh] = 0
	img_org[img[:,:,2] < roi_r_thresh] = 0
	img_org[img[:,:,1] < roi_g_thresh] = 0
	img_org[img_org[:,:,0] > 0] = 255 

	# Define a kernel to smoothen the thresholded image
	kernel = np.ones((7,7), dtype = np.uint8)

	# Obtain a uniform threshold by constricting and dilating the thresholded image
	mask_erode = cv2.erode(img_org[:,:,2], kernel, iterations = 7)
	mask_dilate = cv2.dilate(mask_erode, kernel, iterations = 8)

	# Find the largest contour (sidewalk) in the mask
	(contours,_)= cv2.findContours(mask_dilate.copy(),cv2.RETR_LIST , cv2.CHAIN_APPROX_SIMPLE)
	if (len(contours) > 0):
		c_max = max(contours, key = cv2.contourArea)
		M = cv2.moments(c_max) 
		peri = cv2.arcLength(c_max, True)
		approx = cv2.approxPolyDP(c_max, 0.005 * peri, True)
		cv2.drawContours(img_untouch, [approx], -1, (0, 0, 255), -1)

	return img_untouch









