#!/usr/bin/env python

#from typing_extensions import Self
from pickle import TRUE
import rospy
import json
import math
import datetime
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

#for rappidmq
import pika
import sys


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

class Odom():
    def __init__(self):
        
        #rappidmq
        self.connection_odom = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        
        rospy.init_node('Odom', anonymous=True)

        self.rate = rospy.Rate(10) # 100hz
        self.odom_sub=rospy.Subscriber('odom',Odometry,self.odom_callback)
        self.steer_angle_sub=rospy.Subscriber('steer_angle',Float64,self.steer_angle_callback)

        # state variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.velocity = 0
        self.steer_angle = 0.
        
        
        #rabbitmq
        self.channel_odom = self.connection_odom.channel()
        self.channel_odom.exchange_declare(exchange='topic_logs', exchange_type='topic')
        print("spinning")
        rospy.spin()
        

    def odom_callback(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        _, _, self.theta = quaternion_to_euler( data.pose.pose.orientation.x, 
                                                data.pose.pose.orientation.y, 
                                                data.pose.pose.orientation.z, 
                                                data.pose.pose.orientation.w)
        self.velocity = data.twist.twist.linear.x
        
        rt = rospy.get_rostime()
        rostime = rt.secs + rt.nsecs * 1e-09
        rostimeISO = datetime.datetime.strptime(datetime.datetime.utcfromtimestamp(rostime).isoformat(timespec='milliseconds')+'+0100', "%Y-%m-%dT%H:%M:%S.%f%z")
        
        if not self.connection_odom or self.connection_odom.is_closed:
            self.connection_odom = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
            self.channel_odom = self.connection.channel()
            self.channel_odom.exchange_declare(exchange='topic_logs', exchange_type='topic')
        
        
        routing_key = "fmu.input.odom"
        # create message containing all of the state values from sensor
        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'x_s': self.x,
            'y_s': self.y,
            'theta_s': self.theta,
            'velocity_s': self.velocity,
            'steer_angle_s': self.steer_angle
        }

        self.channel_odom.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))

    def steer_angle_callback(self, data):
        self.steer_angle = data.data
        

if __name__ == '__main__':
    try: 
        Odom()
    except rospy.ROSInterruptException:
        pass