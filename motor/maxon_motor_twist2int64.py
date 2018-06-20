#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64

class Twist2int64():
    def __init__(self):
	    self.command_left = Int64()
        self.command_right = Int64()
        self.received_twist = None
        rospy.init_node('Twist2int64')
        rospy.Subscriber('motor/twist/cmd_vel', Twist, self.callback)
        self.pub_right = rospy.Publisher('right_motor/cmd_vel', Int64, queue_size=10)#name, topic_type, size
        self.pub_left = rospy.Publisher('left_motor/cmd_vel', Int64, queue_size=10)#name, topic_type, size

    def main_twist2int64(self):
        
        rospy.spin()

    def callback(self, message):
        self.received_twist = message #input data
        self.command_right, self.command_left = self.twist2rpm(self.received_twist)
        self.pub_right.publish(self.command_right)
        self.pub_left.publish(self.command_left)

    def twist2rpm(self, received_data):#convert to speed
        #(m/s, rad/s)
        wheeles_size = 0.075#wheel size
        axle_length = 0.35#axle_size(2d)

        v = received_data.linear.x#(m/s)
        omega = received_data.angular.z#(rad/s)

        v_r = (omega*axle_length + 2*v)/2
        v_l = (omega*axle_length - 2*v)/(-2)

        v_r = v_r/(wheeles_size * 2 * 3.14) #wheel_speed(1/s)
        v_l = v_l/(wheeles_size * 2 * 3.14) #wheel_speed(1/s)
        r_rpm = 60 * v_r * 19 #gear rate
        l_rpm = 60 * v_l * 19 #gear rate

        return r_rpm, l_rpm

#Main Program
Convert = Twist2int64()
Convert.main_twist2int64()

#pub memo
	#rostopic pub motor/twist/cmd_vel geometry_msgs/Twist '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
