#!/usr/bin/env python

from pickle import TRUE
import rospy
import json
from ackermann_msgs.msg import AckermannDriveStamped
import pika


class Motor():
    """
    Node representation of the hardware controller on simulator ("physical car")
    Responsible for sending desired speed and steering angle back to the simulator
    """
    def __init__(self):
        #rospy        
        rospy.init_node('Motor', anonymous=True)
    
        self.pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=10)
        self.rate = rospy.Rate(100) # 100hz

        # control inputs
        self.acceleration = 0.
        self.steer_angle_vel = 0.

        #rabbitmq   
        self.connection_fmu = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
                
        self.channel_fmu = self.connection_fmu.channel()
        self.channel_fmu.exchange_declare(exchange='topic_logs', exchange_type='topic')     
        
        result = self.channel_fmu.queue_declare('FMU', exclusive=TRUE)
        queue_name = result.method.queue

        self.channel_fmu.queue_bind(
            exchange = 'topic_logs', queue=queue_name, routing_key="fmu.output")

        self.channel_fmu.basic_consume(
            queue=queue_name, on_message_callback=self.fmu_callback, auto_ack=True)

        print("spinning")
        self.channel_fmu.start_consuming()
       
        

    def fmu_callback(self, ch, method, properties, body):
        """
        Getting desired speed and steering angle computed by the FMU 
        """
        #receiving data
        body = json.loads(body)  

        #save data in variables
        try:
            self.acceleration = body['acceleration']
        except:
            pass

        try:
            self.steer_angle_vel = body['steer_angle_vel']    
        except:
            pass

        #publishing the data to the simulator
        self.msg = AckermannDriveStamped()
        self.msg.drive.speed = self.acceleration
        self.msg.drive.steering_angle = self.steer_angle_vel
        self.pub.publish(self.msg)
        

if __name__ == '__main__':
    try: 
        Motor()
    except rospy.ROSInterruptException:
        pass