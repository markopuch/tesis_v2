#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

if __name__ == "__main__": # Inicio del programa principal

    # Inicializar el nodo
    rospy.init_node('nodo_publicador_point')

    # Declarar del publicador
    topic = 'bottle_confirmation' 
    pub = rospy.Publisher(topic, Bool, queue_size=1)
    
    # Creacion de una instancia (vacia) del mensaje
    pub_msg = Bool()

    # Tiempo de ejecucion del bucle (en Hz)
    rate = rospy.Rate(100)    # 10hz
    while not rospy.is_shutdown():
        # Asignar los valores al mensaje
        pub_msg.data = True

        # Publicar el mensaje
        pub.publish(pub_msg)
        # Esperar
        rate.sleep()
