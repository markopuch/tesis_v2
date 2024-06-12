#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

class Cam(object):
	def __init__(self, topic_name="camera_frame"):
		self.bridge = CvBridge()
		self.image = np.zeros((10,10))
		self.isub = rospy.Subscriber(topic_name, Image, self.image_callback)

	def image_callback(self, img):
		try:
			self.image = self.bridge.imgmsg_to_cv2(img, "bgr8")
		except CvBridgeError as e:
			print(e)

	def get_image(self):
		return self.image
	
def detection(I):
	HSV = cv2.cvtColor(I, cv2.COLOR_BGR2HSV)

	# lower boundary RED color range values; Hue (0 - 10)
	lower1 = np.array([0, 100, 20])
	upper1 = np.array([10, 255, 255])

	# upper boundary RED color range values; Hue (160 - 180)
	lower2 = np.array([160,100,20])
	upper2 = np.array([179,255,255])

	lower_mask = cv2.inRange(HSV, lower1, upper1)
	upper_mask = cv2.inRange(HSV, lower2, upper2)

	full_mask = lower_mask + upper_mask

	# Aplicar un filtro morfológico para eliminar el ruido de la imagen binaria
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
	full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN, kernel)

	# Buscar los contornos en la imagen binaria
	contours, hierarchy = cv2.findContours(full_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	# Dibujar un rectángulo alrededor del contorno más grande (que debe ser la lata de Coca-Cola)
	if len(contours) > 0:
		max_contour = max(contours, key=cv2.contourArea)
		x, y, w, h = cv2.boundingRect(max_contour)
		cv2.rectangle(I, (x, y), (x+w, y+h), (0, 255, 0), 2)

	return I

if __name__ == '__main__':

	# Inicializar el nodo de ROS
	rospy.init_node('camera_node_move')

	# Objeto que se suscribe al topico de la camara
	topic_name = '/usb_cam/image_raw'
	cam = Cam(topic_name)

	# Tópico para publicar una imagen de salida
	topic_pub = 'image_out'
	pubimg = rospy.Publisher(topic_pub, Image, queue_size=10)

	# Frecuencia del bucle principal
	freq = 10
	rate = rospy.Rate(freq)

	# Bucle principal
	while not rospy.is_shutdown():
	
		# Obtener la imagen del tópico de ROS en formato de OpenCV
		I = cam.get_image()
		#print(I.shape)
		
		if len(I.shape) == 3 and I.shape[2] == 3:
			# Realizar algún tipo de procesamiento sobre la imagen
			# Convertir a escala de HSV
			I=detection(I)

			# # Mostrar la imagen con el rectángulo dibujado
			# cv2.imshow("Lata de gaseosa detectada", I)

			# # Esperar al bucle para actualizar
			# cv2.waitKey(1)
			
			# Opcional: publicar la imagen de salida como tópico de ROS
			pubimg.publish(cam.bridge.cv2_to_imgmsg(I,"bgr8"))
		

		rate.sleep()
	
# cv2.destroyAllWindows()
