#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32, String
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
        self.collision_pub = rospy.Publisher("/collision_probability", Float32, queue_size=10)
        self.traffic_color_pub = rospy.Publisher("/traffic_color", String, queue_size=10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # 미리 학습된 모델 로드
        self.confidence_threshold = 0.2  # 신뢰도 임계값
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

            # 신뢰도 필터링 후 충돌 확률 계산 및 신호등 색깔 구분 로직 호출
            filtered_results = self.filter_results_by_confidence(results)
            #self.calculate_collision_probability(frame, filtered_results)
            self.process_traffic_lights(frame, filtered_results)

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

    def filter_results_by_confidence(self, results):
        filtered_results = []
        for result in results:
            filtered_boxes = [box for box in result.boxes if box.conf > self.confidence_threshold]
            filtered_results.append(filtered_boxes)
        return filtered_results

    def process_traffic_lights(self, frame, results):
        height, width, _ = frame.shape
        roi_x1 = int(0.2 * width)
        roi_x2 = int(0.8 * width)
        roi_y1 = int(0.1 * height)
        roi_y2 = int(0.5 * height)

        # 신호등 탐지 결과 처리
        for boxes in results:
            for detection in boxes:
                x1, y1, x2, y2 = map(int, detection.xyxy[0])
                
                # ROI 내부의 바운딩 박스만 처리
                if x1 >= roi_x1 and x2 <= roi_x2 and y1 >= roi_y1 and y2 <= roi_y2:
                    traffic_light_roi = frame[y1:y2, x1:x2]
                    light_color = self.detect_traffic_light_color(traffic_light_roi)
                    rospy.loginfo(f"Traffic light detected at ({x1}, {y1}, {x2}, {y2}) with color: {light_color}")

                    # 신호등 색깔 퍼블리시 및 로그 출력
                    self.traffic_color_pub.publish(light_color)
                    rospy.loginfo(f"Traffic light color: {light_color}")

    def detect_traffic_light_color(self, roi):
        # HSV 색 공간으로 변환
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 색 범위 설정 (예: 빨간색, 노란색, 초록색)
        red_lower = np.array([0, 70, 50])
        red_upper = np.array([10, 255, 255])
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        green_lower = np.array([40, 50, 50])
        green_upper = np.array([90, 255, 255])

        # 마스크 생성
        red_mask = cv2.inRange(hsv_roi, red_lower, red_upper)
        yellow_mask = cv2.inRange(hsv_roi, yellow_lower, yellow_upper)
        green_mask = cv2.inRange(hsv_roi, green_lower, green_upper)

        # 각 색깔의 픽셀 수 계산
        red_pixels = cv2.countNonZero(red_mask)
        yellow_pixels = cv2.countNonZero(yellow_mask)
        green_pixels = cv2.countNonZero(green_mask)

        # 가장 많은 픽셀 수를 가진 색깔을 반환
        if red_pixels > yellow_pixels and red_pixels > green_pixels:
            return "red"
        elif yellow_pixels > red_pixels and yellow_pixels > green_pixels:
            return "yellow"
        elif green_pixels > red_pixels and green_pixels > yellow_pixels:
            return "green"
        else:
            return "unknown"

    #def calculate_collision_probability(self, frame, results):
        height, width, _ = frame.shape
        center_x, center_y = width / 2, height / 2

        # ROI 설정 (화면의 절반 이하)
        roi_y1 = height // 2
        roi_y2 = height

        for boxes in results:
            for detection in boxes:
                x1, y1, x2, y2 = map(int, detection.xyxy[0])
                
                # ROI 내부의 바운딩 박스만 처리
                if y1 >= roi_y1:
                    box_center_x = (x1 + x2) / 2
                    box_center_y = (y1 + y2) / 2
                    box_area = (x2 - x1) * (y2 - y1)

                    # 중앙과의 거리 비율
                    distance_ratio = ((box_center_x - center_x) ** 2 + (box_center_y - center_y) ** 2) ** 0.5 / ((center_x ** 2 + center_y ** 2) ** 0.5)
                    centrality_factor = 1 - distance_ratio

                    # 바운딩 박스 크기 비율
                    box_size_ratio = box_area / (width * height)
                    
                    # 충돌 확률 계산 (곱적용, 가중치 2:1)
                    collision_probability = (2 * centrality_factor + box_size_ratio) / 3
                    rospy.loginfo(f"Collision probability: {collision_probability}")

                    # 충돌 확률 퍼블리시
                    self.collision_pub.publish(collision_probability)

def main():
    yolo_node = YoloNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down YOLO node.")

if __name__ == '__main__':
    main()
