#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError


if __name__ == '__main__':
    
    rospy.init_node('camera_node_move')
    
    bridge = CvBridge()
    # Tópico para publicar una imagen de salida
    topic_pub = '/camera/rgb/image_raw'
    pubimg = rospy.Publisher(topic_pub, Image, queue_size=10)

    # Nombre del archivo de video
    video_name = 'video_20240603_124132.avi'  # Cambia este nombre al nombre del archivo de tu video
    # Abrir el archivo de video
    cap = cv2.VideoCapture(video_name)
    if not cap.isOpened():
        print("Error al abrir el archivo de video")
        exit()
    # Obtener el frame rate del video
    fps = cap.get(cv2.CAP_PROP_FPS)
    wait_time = int(1000 / fps)
    
    # Frecuencia del bucle principal
    freq = 10
    rate = rospy.Rate(freq)

    # Bucle principal
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            print("Fin del video")
            break
        else:
            
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                try:
                    pubimg.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
                except CvBridgeError as e:
                    print(e)

   
        rate.sleep()