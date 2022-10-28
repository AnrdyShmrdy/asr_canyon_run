#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
class RobotController:

    def __init__(self):
        self.vel_pub = rospy.Publisher("/ponce/cmd_vel", Twist, queue_size=10)
        self.left_sub = rospy.Subscriber("/ponce/left", Int32, self.left_callback)
        self.right_sub = rospy.Subscriber("/ponce/right", Int32, self.right_callback)
        self.twistCmd = Twist()
        self.left_right_counter = 0
        self.left_counter=0
        self.right_counter=0
        self.left_feeler = 0
        self.right_feeler = 0
        self.rate = rospy.Rate(20)
    def publishTwistCmd(self):
        self.vel_pub.publish(self.twistCmd)
    def left_callback(self, msg):
        #rospy.loginfo("left_callback: " + str(msg.data))
        self.left_feeler = msg.data
    def right_callback(self, msg):
        #rospy.loginfo("right_callback: " + str(msg.data))
        self.right_feeler = msg.data
    def currentBehavior(self):
        if self.right_feeler == 1 and self.left_feeler == 0:
            self.right_counter+=1
            self.twistCmd.angular.z = -1.1
            self.twistCmd.linear.x = 0.5
            self.publishTwistCmd()
            #print("right_feeler = 1")
        elif self.left_feeler == 1 and self.right_feeler == 0:
            self.left_counter+=1
            self.twistCmd.angular.z = 1.1
            self.twistCmd.linear.x = 0.5
            self.publishTwistCmd()
            #print("left_feeler = 1")
        elif self.left_feeler == 1 and self.right_feeler == 1:
            self.left_right_counter+=1
            if self.left_right_counter >= 3:
                if self.right_counter > self.left_counter:
                    self.twistCmd.angular.z = 4.81 #I think 4.85 is max because it didn't work at that speed 
                elif self.left_counter > self.right_counter:
                    self.twistCmd.angular.z = -4.81 #I think 4.85 is max because it didn't work at that speed
                else:
                    self.twistCmd.angular.z = 4.81 #I think 4.85 is max because it didn't work at that speed
                rospy.Rate(1).sleep
                self.left_right_counter=0
                self.left_counter=0
                self.right_counter=0
            #self.twistCmd.angular.z = 0.0
            self.twistCmd.linear.x = -1.0
            self.publishTwistCmd()
            #print("left_feeler = 1")
        else:
            self.twistCmd.angular.z = 0
            self.twistCmd.linear.x = 1.0
            self.publishTwistCmd()
    def main_loop(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.currentBehavior()
            

def main(args=None):
    rospy.init_node("Robot",anonymous=True)
    robot_controller = RobotController()
    robot_controller.main_loop()

if __name__ == '__main__':
    main()