#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist, TwistStamped

class Suscriptor_cmdvel(object):
    def __init__(self):
        topic = 'cmd_vel'
        self.sub = rospy.Subscriber(topic, Twist, self.callback_point)
        self.cmd_vel_msg = Twist()
        
    def callback_point(self, msg):
        self.cmd_vel_msg = msg

    def get_cmd_vel(self):
        return self.cmd_vel_msg.linear.x, self.cmd_vel_msg.angular.z
    
class Suscriptor_vel(object):
    def __init__(self,topic):
        self.sub = rospy.Subscriber(topic, TwistStamped, self.callback_point)
        self.vel_msg = TwistStamped()
        
    def callback_point(self, msg):
        self.vel_msg = msg

    def get_cmd_vel(self):
        return self.vel_msg.twist, self.vel_msg.header.stamp

if __name__ == "__main__":
    
    try:
        rospy.init_node('node_datacollection_vel') # Inicializar el nodo
        
        path = "/home/utec/data_vel.txt"
        data=open(path,"w")

        #subscribe cmd_vel
        sub_cmdvel = Suscriptor_cmdvel()
        sub_vel = Suscriptor_vel('vel')
        
        #INIT
        rate = rospy.Rate(100)
        t=0.0
        dt=1.0/100
        
        while not rospy.is_shutdown():
                  
            cmd= sub_cmdvel.get_cmd_vel()
            vel,time_stamp_vel = sub_vel.get_cmd_vel()
            print(t, cmd.linear.x, cmd.angular.z, vel.linear.x, vel.angular.z)
            data.write(str(t)+str(time_stamp_vel)+str(cmd.linear.x)+' '+str(cmd.angular.z)+' '+str(vel.linear.x)+' '+str(vel.angular.z)+'\n')
            #wait
            t=t+dt
            rate.sleep()

        data.close()

    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
