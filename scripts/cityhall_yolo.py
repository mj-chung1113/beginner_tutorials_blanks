#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import numpy as np
from ultralytics import YOLO

class YoloNode:
    def __init__(self):
        rospy.init_node('yolo_node', anonymous=True)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)
        self.image_pub = rospy.Publisher("/yoloedImage", Image, queue_size=10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # 미리 학습된 모델 로드
        rospy.loginfo("YOLO node has been started.")

    def callback(self, data):
        try:
            # 압축된 이미지 메시지를 OpenCV 이미지로 변환
            frame = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert image: {e}")
            return
        
        try:
            # YOLO 모델로 이미지 처리
            results = self.model(frame)

            # YOLOv8에서 이미지를 직접 렌더링하는 방식
            annotated_frame = results[0].plot()  # 첫 번째 결과를 가져와 플롯(주석 추가)
        except Exception as e:
            rospy.logerr(f"Error during YOLO processing: {e}")
            return

        try:
            # 결과 이미지를 ROS 메시지로 변환하여 발행
            output_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.image_pub.publish(output_msg)
            rospy.loginfo("Processed frame published.")
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert processed image: {e}")

def main():
    yolo_node = YoloNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLO node.")

if __name__ == '__main__':
    main()