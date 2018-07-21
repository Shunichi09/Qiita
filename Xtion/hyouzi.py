#!/usr/bin/env python
# -*- coding: utf-8 -*-                                                                   

# 動作確認済み
# roslaunch openni2_launch openni2.launch 

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import copy

class Xtion:
    def __init__(self):
        ## ROS関係
        # Publisher
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # これは、ROS_imageをOpencv_imageに変換するのに必要
        self.bridge = CvBridge()
        # Subscriber
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)

    def callback(self,data):# topicを受信するたびに、このコールバック関数が呼ばれる
        # Topicの呼び出し
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.original_image = copy.copy(self.cv_image)

        except CvBridgeError as e:
            print('Cv_Bridge_Error')

        # RGBからgrey_scaleに変換                                                           
        gray_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

        # 画面サイズ変更
        # ウインドウのサイズを変更                                                               
        half_image = cv2.resize(self.original_image, (0,0), fx=0.5, fy=0.5)

        # ウインドウ表示                                                                         
        cv2.imshow("Origin Image", half_image)
        cv2.imshow('Reasult Image', gray_image)
        cv2.waitKey(3)
    
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(half_image, "bgr8"))
        except CvBridgeError as e:
            print('CV_Bridge_Error')

def main(args):
    xtion_disp = Xtion()
    rospy.init_node('image_disp_pub', anonymous=True) # ノード立ち上げ

    try:
        rospy.spin()
    
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)