#!/usr/bin/env python
#for ros
from requests import request
import rospy
import pika
import datetime
import json
from sensor_msgs.msg import LaserScan
import numpy as np

class Lidar():
    def __init__(self):
        
        #rospy
        rospy.init_node('Lidar', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz    
            
        #rabbitmq
        self.connection_fmu = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.sub=rospy.Subscriber('scan', LaserScan, self.lidar_callback)

        #channel to the FMU
        self.channel_fmu = self.connection_fmu.channel()
        self.channel_fmu.exchange_declare(exchange='topic_logs', exchange_type='topic')
            
        print("spinning")
        rospy.spin()

    def lidar_data_processing(self, distances, points_for_average=50):

        number_of_scans = 540
        phi = np.pi/number_of_scans 

        average_distances = np.zeros(540)
        half = int(points_for_average/2)

        for i in range(half):
            distances[i] = distances[half]
            distances[i + 565] = distances[565]
        
        for i in range(540):
            average_distances[i] = np.average(distances[i: i + points_for_average])
            
        # find the longest distance and corresponding angle
        idx = np.argmax(average_distances)
        distance = average_distances[idx]
        angle = phi*idx - np.pi/2

        return distance, angle
        


    def lidar_callback(self, scan):  
        
        #get time with for rabbit msg
        rt = rospy.get_rostime()
        rostime = rt.secs + rt.nsecs * 1e-09
        rostimeISO = datetime.datetime.strptime(datetime.datetime.utcfromtimestamp(rostime).isoformat(timespec='milliseconds')+'+0100', "%Y-%m-%dT%H:%M:%S.%f%z")
        
        #checking if the send channel and connection is open
        if not self.channel_fmu or self.connection_fmu.is_closed:
            self.connection_fmu = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
            self.channel_fmu = self.connection_fmu.channel()
            self.channel_fmu.exchange_declare(exchange='topic_logs', exchange_type='topic')
            
        #input for desired scan data has to be number remember to change
        distances = np.array(scan.ranges[245: 835])
        distance, angle = self.lidar_data_processing(distances)
                 
        routing_key = "fmu.input.targets"
        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'distance': distance,
            'angle': angle,
            }
            
        self.channel_fmu.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))
        
        
if __name__ == '__main__': 
    try: 
        Lidar()
    except rospy.ROSInterruptException:
        pass

    

