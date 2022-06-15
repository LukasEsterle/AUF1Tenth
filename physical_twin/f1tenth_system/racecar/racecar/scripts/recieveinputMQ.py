#!/usr/bin/env python
#for ros
from requests import request
import rospy

#for rappidmq
import pika
import datetime
import json

import numpy as np

from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

class RecieveInputMQ():
    def __init__(self):
        
        rospy.init_node('RecieveInputMQ', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz

        self.msg = AckermannDriveStamped()
        self.msg.header.frame_id = 'frame_id'
        self.desired_velocity = 0.
        self.desired_steer_angle = 0.
        self.msg.header.stamp = 0.

        self.pub = rospy.Publisher('DTdata',AckermannDriveStamped,queue_size=1)
        
        #Connection from broker
        self.connectionFromBroker = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))


        #Channel from broker
        self.channelFromBroker = self.connectionFromBroker.channel()
        self.channelFromBroker.exchange_declare(exchange='topic_logs', exchange_type='topic')     
        

        result = self.channelFromBroker.queue_declare('broker_channel', exclusive=True)
        queue_name = result.method.queue


        self.channelFromBroker.queue_bind(
            exchange = 'topic_logs', queue=queue_name, routing_key="fmu.output.robot")

        
        self.channelFromBroker.basic_consume(
            queue=queue_name, on_message_callback=self.recieve_input_callback, auto_ack=True)

        print("Spinning")
        self.channelFromBroker.start_consuming()



    def recieve_input_callback(self, ch, method, properties, body):
        body = json.loads(body) 
        print(body) 

        try:
            self.desired_velocity = body['desired_velocity']
            print(self.desired_velocity)
        except:
            pass

        try:
            self.desired_steer_angle = body['desired_steer_angle']    
        except:
            pass

        self.msg.header.stamp = rospy.Time.now()
        self.msg.drive.speed = self.desired_velocity
        self.msg.drive.steering_angle = self.desired_steer_angle
        self.pub.publish(self.msg)
        
        
if __name__ == '__main__': 
    try: 
        RecieveInputMQ()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

    

