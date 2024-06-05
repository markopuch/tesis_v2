#!/usr/bin/env python
from math import pi, cos, sin
import rospy
import tf
from std_msgs.msg import Bool
import roboclaw_driver.roboclaw_driver as roboclaw

class Suscriptor_confirmation(object):
    def __init__(self):
        topic = 'bottle_confirmation'
        self.sub = rospy.Subscriber(topic, Bool, self.callback_point)
        self.sub_msg = Bool()
    def callback_point(self, msg):
        self.sub_msg = msg
    def activation_mechanism(self,SPEED):
        bool_confirmation = self.sub_msg.data
        print(bool_confirmation)
        try:
            if (bool_confirmation == True):
                roboclaw.SpeedM1(address, SPEED)
                print("true")
                print(bool_confirmation)
            else:
                roboclaw.ForwardM1(address, 0)
                print("false")
                print(bool_confirmation)
        except OSError as e:
            rospy.logwarn("SpeedM1 OSError: %d", e.errno)
            rospy.logdebug(e) 


if __name__ == "__main__":
    try:
        rospy.init_node('node_roboclaw') # Inicializar el nodo

        #data=open("/home/utec/datar.txt","w")

        rospy.loginfo("Connecting to roboclaw")
        baud_rate = int(rospy.get_param("~baud", "115200"))
        address = int(rospy.get_param("~address", "128"))
        dev_name = rospy.get_param("~dev", "/dev/ttyACM0")
        SPEED = int(rospy.get_param("~speed", "2500")) #ticks/sec
        
        if address > 0x87 or address < 0x80:
            rospy.logfatal("Address out of range")
            rospy.signal_shutdown("Address out of range")
            
        #OPEN ROBOCLAW
        try:
                roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
                rospy.logfatal("Could not connect to Roboclaw")
                rospy.logdebug(e)
                rospy.signal_shutdown("Could not connect to Roboclaw")

        #VERSION
        try:
                version = roboclaw.ReadVersion(address)
        except Exception as e:
                rospy.logwarn("Problem getting roboclaw version")
                rospy.logdebug(e)
                pass

        if not version[0]:
            rospy.logwarn("Could not get version from roboclaw")
        else:
            rospy.logdebug(repr(version[1]))

        #INIT MOTOR
        roboclaw.SpeedM1(address, 0)
        roboclaw.ResetEncoders(address)
        
        rospy.sleep(1)

        rospy.logdebug("dev %s", dev_name)
        rospy.logdebug("baud %d", baud_rate)
        rospy.logdebug("address %d", address)
        print("dev %s", dev_name)
        print("baud %d", baud_rate)
        print("address %d", address)
        
        #ubscribe cmd_vel
        sub = Suscriptor_confirmation()# Crear el suscriptor

        #INIT
        rospy.loginfo("Starting motor drive")
        rate = rospy.Rate(100)
        #t=0.0
        #dt=1.0/100
        while not rospy.is_shutdown():

            enc1 = None

            try:
                enc1= roboclaw.ReadEncM1(address)
            except ValueError:
                pass
            except OSError as e:
                rospy.logwarn("ReadEncM1 OSError: %d", e.errno)
                rospy.logdebug(e)


            speed1 = roboclaw.ReadSpeedM1(address)

            print("Encoder1:"),
            if(enc1[0]==1):
                print (enc1[1]),
            else:
                print ("failed"),

            print ("Speed1:"),
            if(speed1[0]):
                print (speed1[1]),
            else:
                print ("failed"),

            sub.activation_mechanism(SPEED)            
            
            #data.write(str(t)+' '+str(speed1[1])+' '+str(SPEED)+'\n')
            #wait
            #t=t+dt
            rate.sleep()

        #data.close()

    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting")
