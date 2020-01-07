#!/usr/bin/env python
#encoding: utf8
import rospy
import os
import socket
from geometry_msgs.msg import Twist
import subprocess


class JuliusReceiver:
    def __init__(self):
        # juliusとソケット通信を行う
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect(("localhost",10500))
                break
            except:
                rate.sleep()

        rospy.on_shutdown(self.sock.close)
        # 速度をパブリッシュする
        # self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.twist_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
        self.twist = Twist()

    def get_line(self):
        # juliusから受け取ったデータを1行ごとに返す
        line = ""
        while not rospy.is_shutdown():
            v = self.sock.recv(1)
            if v == '\n':
                return line
            line += v

    def score(self,line):
        # "CM="を区切り文字にしてlineを分割し,それぞれの部分を返す
        return float(line.split('CM="')[-1].split('"')[0])

    def pub_command(self,th):
        line = self.get_line()
        rate2 = rospy.Rate(10)

        if "WHYPO" not in line:   return None
        if self.score(line) < th: return None

        if "前" in line:
            # 男性の声で発話
            # cmd1m = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_man.sh","前に進みます"]
            # subprocess.call(cmd1m)
            # 女性の声で発話
            cmd1w = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_woman.sh","前に進みます"]
            subprocess.call(cmd1w)
            rospy.sleep(1)
            # 約3秒直進したあと停止
            self.twist.linear.x = 0.3
            for t in range(30):
                self.twist_pub.publish(self.twist)
                rate2.sleep()
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "後" in line:
            # 男性の声で発話
            # cmd2m = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_man.sh","後ろに進みます"]
            # subprocess.call(cmd2m)
            # 女性の声で発話
            cmd2w = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_woman.sh","後ろに進みます"]
            subprocess.call(cmd2w)
            rospy.sleep(1)
            # 約3秒後退したあと停止
            self.twist.linear.x = -0.3
            for t in range(30):
                self.twist_pub.publish(self.twist)
                rate2.sleep()
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "右" in line:
            # 男性の声で発話
            # cmd3m = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_man.sh","右に回転します"]
            # subprocess.call(cmd3m)
            # 女性の声で発話
            cmd3w = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_woman.sh","右に回転します"]
            subprocess.call(cmd3w)
            rospy.sleep(1)
            # 約3秒右回転したあと停止
            self.twist.angular.z = -0.5
            for t in range(30):
                self.twist_pub.publish(self.twist)
                rate2.sleep()
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "左" in line:
            # 男性の声で発話
            # cmd4m = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_man.sh","左に回転します"]
            # subprocess.call(cmd4m)
            # 女性の声で発話
            cmd4w = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_woman.sh","左に回転します"]
            subprocess.call(cmd4w)
            rospy.sleep(1)
            # 約3秒左回転したあと停止
            self.twist.angular.z = 0.5
            for t in range(30):
                self.twist_pub.publish(self.twist)
                rate2.sleep()
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "こんにちは" in line:
            # 男性の声で発話
            # cmd5m = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_man.sh","こんにちは。お元気ですか。"]
            # subprocess.call(cmd5m)
            # 女性の声で発話
            cmd5w = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_woman.sh","こんにちは。お元気ですか。"]
            subprocess.call(cmd5w)
            rospy.sleep(1)
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "ルンバ" in line:
            # 女性の声で発話
            cmd6w = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_woman.sh","ルンバです"]
            subprocess.call(cmd6w)
            rospy.sleep(1)
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "オッケイタケカワ" in line:
            # 男性の声で発話
            cmd7m = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_man.sh","タケカワです"]
            subprocess.call(cmd7m)
            rospy.sleep(1)
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "かっこいいね" in line:
            # 女性の声で発話
            cmd8w = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_man.sh","ありがとうございます"]
            subprocess.call(cmd8w)
            rospy.sleep(1)
            # 約1回転して停止
            self.twist.angular.z = 1
            for t in range(60):
                self.twist_pub.publish(self.twist)
                rate2.sleep()
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            rospy.sleep(1)

        elif "かわいいね" in line:
            # 女性の声で発話
            cmd9w = ["/home/srd-gtune/catkin_ws/src/ros_voice/src/jtalk_woman.sh","ありがとうございます"]
            subprocess.call(cmd9w)
            rospy.sleep(1)
            # 約1回転して停止
            self.twist.angular.z = -1
            for t in range(60):
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
