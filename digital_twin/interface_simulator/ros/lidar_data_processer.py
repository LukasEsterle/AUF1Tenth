#!/usr/bin/env python
#for ros
from requests import request
import rospy

#for rappidmq
import pika
import datetime
import json

import numpy as np

class LidarDataProcesser():
    def __init__(self):
        
        #rospy
        rospy.init_node('LidarDataProcesser', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz
        
        #rabbitmq
        
        self.credentials = pika.PlainCredentials('guest', 'guest')
        self.connectionToFMU = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.connectionFromRobot = pika.BlockingConnection(pika.ConnectionParameters(host='192.168.1.3', port=5672, credentials=self.credentials))
        """
        self.connectionToFMU = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.connectionFromRobot = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        """
        #channel to the FMU
        self.channelToFmu = self.connectionToFMU.channel()
        self.channelToFmu.exchange_declare(exchange='topic_logs', exchange_type='topic')
            
        #channel from the robot
        self.channelRobotLidar = self.connectionFromRobot.channel()
            
        self.channelRobotLidar.exchange_declare(exchange='topic_logs', exchange_type='topic')     
            
        result = self.channelRobotLidar.queue_declare('RobotToRos', exclusive=True)
        queue_name = result.method.queue


        self.channelRobotLidar.queue_bind(
            exchange = 'topic_logs', queue=queue_name, routing_key="lidar.ranges")

        
        self.channelRobotLidar.basic_consume(
            queue=queue_name, on_message_callback=self.lidar_callback, auto_ack=True)
        
        print("spinning")
        self.channelRobotLidar.start_consuming()
        

    # Function takes ranges from a lidar scan, and uses averaging to find 
    # the angle which is the furthest away based on the average, 
    # and the corresponding distance
    def lidar_data_averageing(self, ranges, points_for_average=50):

            number_of_scans = 540

            # distance between each scan in radians
            phi = np.pi/number_of_scans 

            average_ranges = np.zeros(540)
            half = int(points_for_average/2)

            for i in range(half):
                ranges[i] = ranges[half]
                ranges[i + 565] = ranges[565]
            
            for i in range(540):
                average_ranges[i] = np.average(ranges[i: i + points_for_average])
                
            # find the longest distance and corresponding angle
            idx = np.argmax(average_ranges)
            distance = average_ranges[idx]
            angle = phi*idx - np.pi/2

            return distance, angle

    def lidar_callback(self, ch, method, properties, body):  
        
        #get time with for rabbit msg
        rt = rospy.get_rostime()
        rostime = rt.secs + rt.nsecs * 1e-09
        rostimeISO = datetime.datetime.strptime(datetime.datetime.utcfromtimestamp(rostime).isoformat(timespec='milliseconds')+'+0100', "%Y-%m-%dT%H:%M:%S.%f%z")
        
        #checking if the send channel and connection is open
        if not self.channelToFmu or self.connectionToFMU.is_closed:
            self.connectionToFMU = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
            self.channelToFmu = self.connectionToFMU.channel()
            self.channelToFmu.exchange_declare(exchange='topic_logs', exchange_type='topic')
            
        body = json.loads(body) 
        
        # Only using scans in the filed of view at the front of the car 
        # (so that we don't try to drive to somwhere behind us)
        ranges = body['ranges'][245: 835]

        distance, angle = self.lidar_data_averageing(ranges)
                 
        print(len(body['ranges']))
        
        routing_key = "fmu.input.targets"
        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'distance': distance,
            'angle': angle
            }
            
        self.channelToFmu.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))
        
        
if __name__ == '__main__': 
    try: 
        LidarDataProcesser()

    except rospy.ROSInterruptException:
        pass

    

