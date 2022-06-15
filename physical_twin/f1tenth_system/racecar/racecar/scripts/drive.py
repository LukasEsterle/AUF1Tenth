#!/usr/bin/env python

from pickle import TRUE
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy

class Brain():
    def __init__(self):
        
        rospy.init_node('Brain', anonymous=True)
        self.rate = rospy.Rate(100) # 10hz

        #SPEED LIMITER
        self.speed_limit = 2 

        self.current_speed = 0
        self.EMERGENCY_STOP = False
        self.MANUAL_CONTROL = False
        self.ppControl = False
        self.scantime_interval = rospy.Duration(secs=0.2)
        self.last_scan = rospy.Time.now()
        self.ttc_threshold = 0.4

        #Subscribing to lidar data from scan topic
        self.laserSub = rospy.Subscriber('/scan', LaserScan, self.emergency_brake)

        #Subscribing to desired speed and steering angle data recieved from DT
        self.twinSub = rospy.Subscriber('/DTdata', AckermannDriveStamped, self.digital_twin_control)   

        #Subscribing to buttons data from joy to check if manual control is ON
        self.joySub = rospy.Subscriber('/vesc/joy', Joy, self.control_switch)

        #Subscribing to desired speed and steering angle data recieved from manual_controller
        self.mcSub = rospy.Subscriber('/MCdata', AckermannDriveStamped, self.manual_control)

        #Subscribing to desired speed and steering angle data recieved from Pure Pursuit
        self.ppSub = rospy.Subscriber('/ppdata',AckermannDriveStamped,self.pure_pursuit)

        #Publishing to Teleop to control the car
        self.pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/input/teleop',AckermannDriveStamped,queue_size=1)
        

    def emergency_brake(self, scan):  
        self.ranges = scan.ranges
        # TIME TO COLLISION SHOULD BE IMPLEMENTED HERE
        if self.current_speed > 0 and rospy.Time.now()-self.last_scan > self.scantime_interval: #TTC -> inf if dist deriv = 0
            ttcLHS = []
            ttcRHS = []
            for index, p in enumerate(self.ranges[360:540]): #45degree LHS
                if not math.isinf(p) and not math.isnan(p):
                    angle = math.radians(45)-scan.angle_increment*index
                    distance_deriv = self.current_speed*math.cos(angle)
                    self.last_scan = rospy.Time.now()
                    if (distance_deriv > 0): #Ensure no division by 0
                        ttcLHS.append(p/distance_deriv)

        #Tests for both lhs and rhs, sign doesnt matter for cosine calculations
            for index, p in enumerate(self.ranges[540:720]): #45degree RHS
                if not math.isinf(p) and not math.isnan(p):
                    angle = math.radians(45)-scan.angle_increment*index
                    distance_deriv = self.current_speed*math.cos(angle)
                    if (distance_deriv > 0): #Ensure no division by 0
                        ttcRHS.append(p/distance_deriv)
            ttc = ttcLHS + ttcRHS
            
            try:
                minTTC = np.min(ttc)
            except ValueError:
                pass
            if (minTTC > 0 and np.min(ttc) < self.ttc_threshold):
                rospy.loginfo("AUTOMATIC BREAKING ENGAGED")
                self.drive(0, 0)
                self.EMERGENCY_STOP = True 
            else:
                self.EMERGENCY_STOP = False


    def drive(self, desired_speed, desired_steering_angle):
        # SPEED LIMITER
        if (desired_speed > self.speed_limit):    #if above speed limit
            self.current_speed = self.speed_limit
        elif (desired_speed < -self.speed_limit): #if below negative speed limit - twin will never, but controller might
            self.current_speed = -self.speed_limit
        else:
            self.current_speed = desired_speed

        self.steering_angle = desired_steering_angle
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.Time.now()
        ack_msg.header.frame_id = "frame_id"
        ack_msg.drive.speed = self.current_speed
        ack_msg.drive.steering_angle = self.steering_angle
        self.pub.publish(ack_msg)

    def control_switch(self, data):
        if data.buttons[4] == 1: #L1
            self.MANUAL_CONTROL = True
        if data.buttons[4] == 0:
            self.MANUAL_CONTROL = False
            self.drive(0,0)
        if data.buttons[7] == 1: #R2
            rospy.loginfo("Disabled emergency brake")
            self.EMERGENCY_STOP = False
        if data.buttons[2] == 1: #circle
            self.ppControl = True
        if data.buttons[2] == 0:
            self.ppControl = False
        if data.buttons[3] == 1: #triangle
            self.EMERGENCY_STOP = True
            self.drive(0,0)
    
    
    def digital_twin_control(self, data):
        if not self.EMERGENCY_STOP and not self.MANUAL_CONTROL and not self.ppControl:
            rospy.loginfo("DT enganged")
            self.drive(data.drive.speed, data.drive.steering_angle)

    def manual_control(self, data):
        if self.MANUAL_CONTROL and not self.EMERGENCY_STOP:
            rospy.loginfo("ManualControl enganged")
            self.drive(data.drive.speed, data.drive.steering_angle)

    def pure_pursuit(self,data):
        if self.ppControl and not self.EMERGENCY_STOP and not self.MANUAL_CONTROL:
            rospy.loginfo("PurePursuit enganged")
            self.drive(data.drive.speed,data.drive.steering_angle)

if __name__ == '__main__': 
    try: 
        Brain()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        
