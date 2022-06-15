#!/usr/bin/env python
import rospy 
import json
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
class PurePursuit:
    def __init__(self):
        rospy.init_node("Pure_Pursuit",anonymous=True)
        self.rate = rospy.Rate(100)
        self.laserSub = rospy.Subscriber('/scan',LaserScan,self.callback)
        self.ppPub = rospy.Publisher('/ppdata',AckermannDriveStamped,queue_size=1)
    
    """
    Publishes ackermann msg with angle and speed. 
    Speed is at max at [15:30] m, 30 being max value.  
    otherwise equal to 0.3x distance
    Could do non-linear scaling
    """
    def publish_drive(self,distance,angle):
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id='Pure Pursuit Data'
        ack_msg.drive.speed = 0.33 * distance
        ack_msg.drive.steering_angle = angle
        self.pub.publish(ack_msg)

    """
    Callback function for laserscan.
    Takes LaserScan message, points for average, and number of scans.
    """
    def callback(self,scan,pfa = 50,scanAmount = 300):
        self.scanranges = np.asarray(scan.ranges) #Convert from tuple to array
        half_scan_amount = int(scanAmount / 2)
        #Angular resolution
        phi = scan.angle_increment
        #Half the pfa (for each side of fov)
        half_pfa = int(pfa / 2)

        #Take the subset from centered around point 540 (lidar FOV centerpoint). 
        ranges = self.scanranges[540-half_scan_amount-half_pfa:540+half_scan_amount+half_pfa]
        avrg_ranges = [0]*scanAmount

        #set the fpa (overflow protection essentially) to same length as first and last scan in fov. 
        for i in range(half_pfa):
            avrg_ranges[i] = ranges[half_pfa]
            ranges[i + scanAmount + half_pfa] = ranges[scanAmount + half_pfa]

        #for the other values in fov, take the average of the pfa neighbours. 
        for i in range(scanAmount):
            avrg_ranges[i] = np.average(ranges[i: i + scanAmount])
        #get the index and length of the largest value (average length)
        idx_furthest = np.argmax(avrg_ranges)
        length = avrg_ranges[idx_furthest]
        #Calculate the angle wrt. car orientation
        angle = phi * idx_furthest + phi*((1080-scanAmount)/2)-np.radians(135)
        #normalize (max value1 / maxvalue2)
        angle_norm = 0.34 / (np.pi/2) 
        #Length does not need to be normalized here.
        self.publish_drive(length,angle*angle_norm)

if __name__ == '__main__': 
    try: 
        PurePursuit()
        print("Spinning")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
