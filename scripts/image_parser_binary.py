#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np
import os, rospkg

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridgeError

class IMGParser:
    def __init__(self):
        rospy.init_node('image_parser', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.is_image = False

        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            os.system('clear')
            if not self.is_image:
                print("[1] can't subscribe '/image_jpeg/compressed' topic... \n    please check your Camera sensor connection")
            else:
                print(f"Caemra sensor was connected !")

            rate.sleep()


    def callback(self, msg):
        self.is_image = True
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    
        img_hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)

        lower_wlane = np.array([0,00,90])
        upper_wlane = np.array([30,60,255])

        if np.sum(lower_wlane) == 0:
            print("튜닝 다시해!!")
            exit()
        
        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)

        img_wlane = cv2.cvtColor(img_wlane, cv2.COLOR_GRAY2BGR)

        img_concat = np.concatenate([self.img_bgr, img_hsv,img_wlane], axis =1)
        
        #cv2.imshow("RGB", img_bgr)
        #cv2.waitKey(1)
        #cv2.imshow("HSV", img_hsv)
        #cv2.waitKey(1)
        cv2.imshow("CONCAT", img_concat)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        IMGParser = IMGParser()
    except rospy.ROSInterruptException:
        pass
