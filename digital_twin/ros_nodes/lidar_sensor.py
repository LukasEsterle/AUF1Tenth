#!/usr/bin/env python3
#for ros
from requests import request
import rospy
import pika
import datetime
import json
from sensor_msgs.msg import LaserScan
import numpy as np

class Lidar():
    """
    Node receiving the lidar data 
    """
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



    def lidar_callback(self, scan):  
        """
        Callback for receving lidar data
        """

        #timestamps for rabbit msg
        rt = rospy.get_rostime()
        rostime = rt.secs + rt.nsecs * 1e-09
        rostimeISO = datetime.datetime.strptime(datetime.datetime.utcfromtimestamp(rostime).isoformat(timespec='milliseconds')+'+0100', "%Y-%m-%dT%H:%M:%S.%f%z")
        
        #checking if the send channel and connection is open
        if not self.channel_fmu or self.connection_fmu.is_closed:
            self.connection_fmu = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
            self.channel_fmu = self.connection_fmu.channel()
            self.channel_fmu.exchange_declare(exchange='topic_logs', exchange_type='topic')

        #receiving the data
        ranges = np.array(scan.ranges).tolist()
                 
        #creating message to send to lidar processing node 
        routing_key = "lidar.ranges"
        message = {
            'time': rostimeISO.isoformat(timespec='milliseconds'),         
            'ranges': ranges
            }
        
        #conversion from python string to json string 
        self.channel_fmu.basic_publish(
            exchange='topic_logs', routing_key=routing_key, body=json.dumps(message))
        
        
if __name__ == '__main__': 
    try: 
        Lidar()
    except rospy.ROSInterruptException:
        pass

    