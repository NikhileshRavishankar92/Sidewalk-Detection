#!/usr/bin/env python
from __future__ import print_function
import rospy
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
import numpy as np 
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
from Robot_vision import sidewalk_detection

class cvBridgeDemo():
	def __init__(self):
		self.node_name = "sidewalk_detect"

		# Initialize ROS node
		rospy.init_node(self.node_name)
		# During Shutdown
		rospy.on_shutdown(self.cleanup)


		# Create the cv_bridge object
		self.bridge = CvBridge()

		# Subscribe to camera image and depth topics and set the appropriate callbacks
		self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
		#self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image,self.depth_callback)

		rospy.loginfo("Waiting for image topics...")


	def image_callback(self, ros_image):
	# Use cv_bridge() to convert the ROS image to OpenCV format
		try:
			frame = self.bridge.imgmsg_to_cv2(ros_image,"bgr8")
		except CvBridgeError as e:
			print (e) 

		#Convert image to numpy array
		frame = np.array(frame, dtype = np.uint8)

		#Process the frame
		display_image = self.process_image(frame)

		#Display the image
		cv2.imshow("Window",display_image)

		#Process keyboard command
		self.keystroke = cv2.waitKey(1)
		if 32 <= self.keystroke and self.keystroke < 128:
			cc = chr(self.keystroke).lower()

			if (cc == 'q'):
				# Exit
				rospy.signal_shutdown("User has hit q to exit")

	def process_image(self,frame):
		# Invert the image	
		frame = cv2.flip(frame,0)
		frame = cv2.flip(frame,1)
		result = sidewalk_detection(frame)
		return result

	def cleanup(self):
		print ("Shutting down vision node")
		cv2.destroyAllWindows()

def main(args):
	try:
		cvBridgeDemo()
		rospy.spin()
	except KeyboardInterrupt:
		print ("Shutting down vision node")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)






