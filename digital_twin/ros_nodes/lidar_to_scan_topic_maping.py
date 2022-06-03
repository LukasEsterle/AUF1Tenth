#!/usr/bin/env python
# for ros
import datetime
import json
from matplotlib.pyplot import sca

import numpy as np
# for rappidmq
import pika
import rospy
from requests import request
from sensor_msgs.msg import LaserScan


class LidarDataProcesser:
    def __init__(self):

        # rospy
        rospy.init_node("LidarDataProcesser", anonymous=True)
        self.rate = rospy.Rate(100)  # 100hz

        # rabbitmq
        
        	
        self.credentials = pika.PlainCredentials("guest", "guest")
        self.connectionFromRobot = pika.BlockingConnection(
            pika.ConnectionParameters(
                host="192.168.1.3", port=5672, credentials=self.credentials
            )
        )
        
        #self.connectionFromRobot = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        
        
        # channel from the robot
        self.channelRobotLidar = self.connectionFromRobot.channel()

        self.channelRobotLidar.exchange_declare(
            exchange="topic_logs", exchange_type="topic"
        )

        result = self.channelRobotLidar.queue_declare(
            "RobotToRos", exclusive=True)
        queue_name = result.method.queue

        self.channelRobotLidar.queue_bind(
            exchange="topic_logs", queue=queue_name, routing_key="lidar.ranges"
        )

        self.channelRobotLidar.basic_consume(
            queue=queue_name, on_message_callback=self.lidar_callback, auto_ack=True
        )

        print("spinning")
        self.channelRobotLidar.start_consuming()
    
    def lidar_callback(self, ch, method, properties, body):
        
        pup = rospy.Publisher("scan2", LaserScan, queue_size= 10)
        
        scan = LaserScan()
        
        data = json.loads(body)
        
        scan.header.seq = data["seq"]
        scan.header.stamp.secs = data["secs"]
        scan.header.stamp.nsecs = data["nsecs"]
        scan.header.frame_id = data["frame_id"]
        scan.angle_min = data["angle_min"]
        scan.angle_max = data["angle_max"]
        scan.angle_increment = data["angle_increment"]
        scan.time_increment = data["time_increment"]
        scan.scan_time = data["scan_time"]
        scan.range_min = data["range_min"]
        scan.range_max = data["range_max"]
        scan.ranges = data["ranges"]
        scan.intensities = data["intensities"]
        
        pup.publish(scan)
        
        
        
        
        


if __name__ == "__main__":
    try:
        LidarDataProcesser()

    except rospy.ROSInterruptException:
        pass
