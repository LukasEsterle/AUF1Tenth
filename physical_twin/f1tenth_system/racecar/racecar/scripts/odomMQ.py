# /vesc/low_level/ackermann_cmd_mux/output
# steering_angle
# speed 


# /vesc/odom
# Position (x og y)
# Heading angle

#!/usr/bin/env python
#for ros
#from requests import request
import rospy

#for rabbitmq
import pika
import datetime
import json

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

import math


# We need to send orientation, which the Digital Twin team
# needs on a specific format. So we use this function to
# convert it to euler before sending it. 
# Note that this quaternion_to_euler function is not ours, 
# it is code from the Digital Twin team.
def quaternion_to_euler(x, y, z, w):        
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))        
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))        
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))        
    X = math.atan2(t0, t1)
    Y = math.asin(t2)
    Z = math.atan2(t3, t4)        
    return X, Y, Z



class OdomMQ():
    def __init__(self):
        
        rospy.init_node('OdomMQ', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz

        self.x = 0
        self.y = 0   
        self.heading_angle = 0
        self.steer_angle = 0
        self.speed = 0
        
        #Connection to broker
        self.connectionToBroker = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))

        #Channel to broker
        self.channelToBroker = self.connectionToBroker.channel()
        self.channelToBroker.exchange_declare(exchange='topic_logs', exchange_type='topic')

        #Subscribing to Odometry
        self.sub=rospy.Subscriber('/vesc/odom', Odometry, self.odom_callback)       

        #Subscribing to AckermannDriveStamped
        self.sub2=rospy.Subscriber('/vesc/low_level/ackermann_cmd_mux/output', AckermannDriveStamped, self.drive_callback)   

        

    def odom_callback(self, odom):  

        #get time with for rabbit msg
        rt = rospy.get_rostime()
        rostime = rt.secs + rt.nsecs * 1e-09
        rostimeISO = datetime.datetime.strptime(datetime.datetime.utcfromtimestamp(rostime).isoformat(timespec='milliseconds')+'+0100', "%Y-%m-%dT%H:%M:%S.%f%z")
        
        #checking if the send channel and connection is open
        if not self.channelToBroker or self.connectionToBroker.is_closed:
            self.channelToBroker = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
            self.channelToBroker = self.connectionToBroker.channel()
            self.channelToBroker.exchange_declare(exchange='topic_logs', exchange_type='topic')

        #Getting position from Odometry
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y

        #Getting orientation from Odometry
        _, _, self.heading_angle = quaternion_to_euler( odom.pose.pose.orientation.x, 
                                                        odom.pose.pose.orientation.y, 
                                                        odom.pose.pose.orientation.z, 
                                                        odom.pose.pose.orientation.w)

        
        routing_key = "fmu.input.odom"
        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'x_s': self.x,
            'y_s': self.x,
            'theta_s': self.heading_angle,
            'velocity_s': self.speed,
            'steer_angle_s': self.steer_angle,
            }
            
        self.channelToBroker.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))


    def drive_callback(self, drive):  
        #Getting steering_angle from AckermannDriveStamped
        self.steer_angle = drive.drive.steering_angle

        #Getting speed from AckermannDriveStamped
        self.speed = drive.drive.speed


        
if __name__ == '__main__': 
    try: 
        OdomMQ()
        print("Spinning")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
