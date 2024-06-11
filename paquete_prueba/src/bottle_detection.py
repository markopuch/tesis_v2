#!/usr/bin/env python3
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
			print(e)

	def get_image(self):
		return self.image
	
def detection(I,kernel=35,th=140,kernel2=7,iter=5):
    
	gray=cv2.cvtColor(I,cv2.COLOR_BGR2GRAY)
	gauss_filter = cv2.GaussianBlur(gray,(kernel,kernel),5)
	
	# Crop the image
	height = 880
	top=400
	polygons = np.array([[(250, height), (1500, height), (1200,top),(400, top)]])
	roi = np.zeros_like(gauss_filter)
	cv2.fillPoly(roi, polygons, 255)
	mask_roi=cv2.bitwise_and(gauss_filter, roi)
	
	# Threshold the image
	_,bw = cv2.threshold(mask_roi, th, 255, cv2.THRESH_BINARY)
	k=np.ones((kernel2, kernel2), np.uint8)
	erode = cv2.erode(bw, k, iterations=iter) 
	
	return erode

def bottle_confirmed(I,count_ones,count_zeros,start_time):
    # Comprobar si el frame procesado es todo negro o tiene pixeles en blanco
    if cv2.countNonZero(I) > 0:
        count_ones += 1
        count_zeros = 0
        msg=True
        start_time = None
    else:
        if count_ones != 0:  # ajusta estos números según tus necesidades
            count_zeros += 1
            msg=True
            if start_time is None:
                start_time = time.time()   
            elif time.time() - start_time > 3:
                msg=False
                start_time = None
                count_ones = 0
                count_zeros = 0
        else:
            start_time = None
            msg=False
            count_zeros = 0
    
    return msg,count_ones,count_zeros,start_time
        

if __name__ == '__main__':

	# Inicializar el nodo de ROS
	rospy.init_node('camera_node_move')

	# Objeto que se suscribe al topico de la camara
	topic_name = '/camera/rgb/image_raw'
	cam = Cam(topic_name)

	# Tópico para publicar una imagen de salida
	topic_pub = 'image_proccesed'
	pubimg = rospy.Publisher(topic_pub, Image, queue_size=10)

	# Tópico para publicar un booleano de salida 
	topic = 'bottle_confirmation' 
	pub = rospy.Publisher(topic, Bool, queue_size=1)
	# Creacion de una instancia (vacia) del mensaje
	pub_msg = Bool()
	pub_msg.data=False
	# Inicializar el contador de unos y ceros
	count_ones = 0
	count_zeros = 0
	# Inicializar la variable de tiempo de inicio
	start_time = None

	# Frecuencia del bucle principal
	freq = 10
	rate = rospy.Rate(freq)
 
	# Bucle principal
	while not rospy.is_shutdown():
	
		# Obtener la imagen del tópico de ROS en formato de OpenCV
		I = cam.get_image()
		
		if len(I.shape) == 3 and I.shape[2] == 3:
			# Realizar algún tipo de procesamiento sobre la imagen
			I=detection(I)

			# Mostrar la imagen con el rectángulo dibujado
			cv2.imshow("Bottella detectada", I)

			pub_msg.data,count_ones,count_zeros,start_time=bottle_confirmed(I,count_ones,count_zeros,start_time)
   
			# Esperar al bucle para actualizar
			cv2.waitKey(1)
			
			# Opcional: publicar la imagen de salida como tópico de ROS
			pubimg.publish(cam.bridge.cv2_to_imgmsg(I,"bgr8"))
		
		# Publicar el mensaje
		pub.publish(pub_msg)

		rate.sleep()
	
cv2.destroyAllWindows()