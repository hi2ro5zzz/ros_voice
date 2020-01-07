#!/usr/bin/env python
#encoding: utf8
import rospy
import os
import socket
from geometry_msgs.msg import Twist


class JuliusReceiver:
    def __init__(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect(("localhost",10500))
                break
            except:
                rate.sleep()

        rospy.on_shutdown(self.sock.close)

        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.twist = Twist()

    def get_line(self):
        line = ""
        while not rospy.is_shutdown():
            v = self.sock.recv(1)
            if v == '\n':
                return line
            line += v

    def score(self,line):
        return float(line.split('CM="')[-1].split('"')[0])

    def pub_command(self,th):
        line = self.get_line()
        rate2 = rospy.Rate(10)

        if "WHYPO" not in line:   return None
        if self.score(line) < th: return None

        if "前" in line:
            self.twist.linear.x = 0.3
            for t in range(30):
                self.twist_pub.publish(self.twist)
                rate2.sleep()
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "後" in line:
            self.twist.linear.x = -0.3
            for t in range(30):
                self.twist_pub.publish(self.twist)
                rate2.sleep()
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "右" in line:
            self.twist.angular.z = -0.5
            for t in range(30):
                self.twist_pub.publish(self.twist)
                rate2.sleep()
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "左" in line:
            self.twist.angular.z = 0.5
            for t in range(30):
                self.twist_pub.publish(self.twist)
                rate2.sleep()
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        rospy.loginfo(self.twist)
        self.twist_pub.publish(self.twist)

if __name__ == '__main__':
    rospy.init_node("voice_twist")
    j = JuliusReceiver()
    while not rospy.is_shutdown():
        j.pub_command(0.999)
