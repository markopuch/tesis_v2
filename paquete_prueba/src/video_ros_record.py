#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import time

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

if __name__ == '__main__':
    
    rospy.init_node('camera_node_1')
    
    # Objeto que se suscribe al topico de la camara
    topic_name = '/usb_cam/image_raw'
    cam = Cam(topic_name)
    
    # Definir el objeto VideoWriter
    # Obtener el ancho y alto de los fotogramas de la camara
    frame_width = 1920
    frame_height = 1080
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    name="video_"+timestamp+".avi"
    out = cv2.VideoWriter(name, fourcc, 20.0, (frame_width, frame_height))

    # Frecuencia del bucle principal
    freq = 10
    rate = rospy.Rate(freq)

    # Bucle principal
    rospy.logdebug("Starting recording node")
    while not rospy.is_shutdown():
        I = cam.get_image()
        
        if len(I.shape) == 3 and I.shape[2] == 3:
            try:
                # Escribir el frame en el archivo de video
                out.write(I)
                
            except CvBridgeError as e:
                rospy.logdebug(e)
        
        rate.sleep()
    # Liberar el objeto VideoWriter al final
    out.release()
