#!/usr/bin/env python3
# for ros
import datetime
import json

import numpy as np
# for rappidmq
import pika
import rospy
from requests import request


class LidarDataProcesser:
    """
    Node processing Lidar data
    """

    def __init__(self):

        # rospy 
        rospy.init_node("LidarDataProcesser", anonymous=True)
        self.rate = rospy.Rate(100)  # 100hz

        # rabbitmq
        """
        self.credentials = pika.PlainCredentials("guest", "guest")
        self.connectionToFMU = pika.BlockingConnection(
            pika.ConnectionParameters(host="localhost")
        )
        self.connectionFromRobot = pika.BlockingConnection(
            pika.ConnectionParameters(
                host="192.168.1.3", port=5672, credentials=self.credentials
            )
        )
        """

        #two channels needed: 
        #1. to receive the raw lidar data from the lidar sensor node 
        #2. to send the processed data to the fmu 
        self.connectionToFMU = pika.BlockingConnection(pika.ConnectionParameters(host='localhost')) #using localhost since "physical car" is the simulator
        self.connectionFromRobot = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        
        # channel to the FMU
        self.channelToFmu = self.connectionToFMU.channel()
        self.channelToFmu.exchange_declare(
            exchange="topic_logs", exchange_type="topic")

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

    

    def lidar_data_averageing(self, body, points_for_average=50, number_of_scans=540):
        """ 
        Function takes ranges from a lidar scan and uses averaging to find
        the angle which is the furthest away based on the average,
        and the corresponding distance.
        """

        #Half the number of scans
        half_nos = int(number_of_scans / 2)
        
        #Distance between each scan in radians
        phi = (np.pi*2 / 1080) # TODO: 3/4 pi because the lidar only covers 270 degrees

        #Half the number of points for average
        half_pfa = int(points_for_average / 2)

        #Only using scans in the field of view at the front of the car
        #(so that we don't try to drive to somewhere behind us) 
        ranges = body["ranges"][half_nos - half_pfa: number_of_scans + half_nos + half_pfa]

        #Set points distance of the points at the left and right side of the field of view,
        #equal to the left and right scan distance, respectively. 
        average_ranges = np.zeros(number_of_scans)

        for i in range(half_pfa):
            ranges[i] = ranges[half_pfa]
            ranges[i + number_of_scans +
                   half_pfa] = ranges[number_of_scans + half_pfa]

        #Compute the average around each point for N neighbours.
        for i in range(number_of_scans):
            average_ranges[i] = np.average(ranges[i: i + points_for_average])

        #Find the point with longest average distance and corresponding angle
        idx = np.argmax(average_ranges)
        distance = average_ranges[idx]
        angle = phi * idx - np.pi / 2

        return distance, angle


    def lidar_callback(self, ch, method, properties, body):
        """
        Callback to receive Lidar data
        """

        # timestamps for rabbit msg
        rt = rospy.get_rostime()
        rostime = rt.secs + rt.nsecs * 1e-09
        rostimeISO = datetime.datetime.strptime(
            datetime.datetime.utcfromtimestamp(rostime).isoformat(
                timespec="milliseconds"
            )
            + "+0100",
            "%Y-%m-%dT%H:%M:%S.%f%z",
        )

        # checking if the send channel and connection is open
        if not self.channelToFmu or self.connectionToFMU.is_closed:
            self.connectionToFMU = pika.BlockingConnection(
                pika.ConnectionParameters(host="localhost")
            )
            self.channelToFmu = self.connectionToFMU.channel()
            self.channelToFmu.exchange_declare(
                exchange="topic_logs", exchange_type="topic"
            )

        #lidar data 
        body = json.loads(body)

        #process data 
        distance, angle = self.lidar_data_averageing(body, points_for_average=50)

        #create message to fmu 
        routing_key = "fmu.input.targets"
        message = {
            "time": rostimeISO.isoformat(timespec="milliseconds"),
            "distance": distance,
            "angle": angle,
        }

        #converts python string to json string -> send to Lidar FMU 
        self.channelToFmu.basic_publish(
            exchange="topic_logs", routing_key=routing_key, body=json.dumps(message)
        )



if __name__ == "__main__":
    try:
        LidarDataProcesser()
    except rospy.ROSInterruptException:
        pass