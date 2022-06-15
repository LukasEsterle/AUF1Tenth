#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

class ManualController():
    def __init__(self):
        rospy.init_node('ManualController', anonymous=True)
        self.rate = rospy.Rate(100)#10hz
        self.msg = AckermannDriveStamped()
        self.msg.header.frame_id = 'frame_id'
        self.msg.header.stamp = 0.

        self.desired_velocity = 0.
        self.desired_steer_angle = 0.

        #Subscribing to buttons data from joy to check if manual control is ON
        self.joySub = rospy.Subscriber('/vesc/joy', Joy, self.pub_manual_control)

        #Publishing to Teleop to control the car
        self.pub = rospy.Publisher('MCdata',AckermannDriveStamped,queue_size=1)


    def pub_manual_control(self, data):  

        self.desired_velocity = data.axes[1] * 5
        self.desired_steer_angle = data.axes[2] * 0.340000003575

      
        self.msg.header.stamp = rospy.Time.now()
        self.msg.drive.speed = self.desired_velocity
        self.msg.drive.steering_angle = self.desired_steer_angle
        self.pub.publish(self.msg)


if __name__ == '__main__': 
    try: 
        ManualController()
        print('ManualController initialized')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
