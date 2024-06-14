#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import time
from std_msgs.msg import Bool

class Cam(object):
	def __init__(self, topic_name="camera_frame"):
		self.bridge = CvBridge()
		self.image = np.zeros((10,10))
		self.isub = rospy.Subscriber(topic_name, Image, self.image_callback)

	def image_callback(self, img):
		try:
			self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			rospy.logdebug(e)
			print(e)

	def get_image(self):
		return self.image
	
def detection(I,kernel=35,th=140,kernel2=7,iter=5):
    
    height = 1020
    top=600
    polygons = np.array([[(250, height), (1500, height), (1250,top),(400, top)]])
    n=I.copy()
    square=cv2.polylines(n, [polygons], isClosed=True, color=(255, 0, 0), thickness=5)
    
    return square

        

if __name__ == '__main__':

	# Inicializar el nodo de ROS
	rospy.init_node('camera_node_image_proccesing_1')

 
	# Objeto que se suscribe al topico de la camara
	topic_name = '/usb_cam/image_raw'
	cam = Cam(topic_name)

	# Topico para publicar una imagen de salida
	topic_pub = 'image_proccesed_squared'
	pubimg = rospy.Publisher(topic_pub, Image, queue_size=10)


	# Frecuencia del bucle principal
	freq = 10
	rate = rospy.Rate(freq)

	# Bucle principal
	rospy.logdebug("Starting image processing node_1")
	while not rospy.is_shutdown():
		# Obtener la imagen del topico de ROS en formato de OpenCV
		I = cam.get_image()
		try:
			if len(I.shape) == 3 and I.shape[2] == 3:
				# Realizar algun tipo de procesamiento sobre la imagen
				I=detection(I)
				# publicar la imagen de salida como topico de ROS
				pubimg.publish(cam.bridge.cv2_to_imgmsg(I,"bgr8"))
		except CvBridgeError as e:
			rospy.logdebug(e)
			print(e)
		

		rate.sleep()
  