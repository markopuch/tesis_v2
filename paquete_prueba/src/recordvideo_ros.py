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

if __name__ == '__main__':
    
    rospy.init_node('camera_node_move')
    
    # Objeto que se suscribe al topico de la camara
    topic_name = '/camera/rgb/image_raw'
    cam = Cam(topic_name)
    
    # Tópico para publicar una imagen de salida
    topic_pub = 'image_out'
    pubimg = rospy.Publisher(topic_pub, Image, queue_size=10)

    # Frecuencia del bucle principal
    freq = 10
    rate = rospy.Rate(freq)

    # Bucle principal
    while not rospy.is_shutdown():
        I = cam.get_image()
        
        if len(I.shape) == 3 and I.shape[2] == 3:
            try:
                pubimg.publish(cam.bridge.cv2_to_imgmsg(I, "bgr8"))
            except CvBridgeError as e:
                print(e)
        
        rate.sleep()