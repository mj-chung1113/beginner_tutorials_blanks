#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from re import I
import rospy
import rospkg
from math import sqrt, radians, sin, cos, atan2
from morai_msgs.msg import GPSMessage


class pathMaker :
    
    def __init__(self, pkg_name, path_name):
        rospy.init_node('path_maker', anonymous=True)
        # /turtle1/pose 토픽 구독
        rospy.Subscriber("/gps", GPSMessage, self.status_callback)
        # 초기화
        self.prev_x = 0
        self.prev_y = 0
        self.is_status=False
        # 패키지 경로 로드 & 파일 쓰기 모드
        rospack = rospkg.RosPack()
        pkg_path=rospack.get_path(pkg_name)
        full_path=pkg_path + '/'+path_name+'.txt'
        self.f=open(full_path, 'w')

        while not rospy.is_shutdown():
            if self.is_status==True :
                # turtle 위치 기록
                self.path_make()
        self.f.close()
    
    def path_make(self):
        x = round(self.status_msg.latitude, 6)
        y = round(self.status_msg.longitude, 6)

        z=0.350974
        distance=pathMaker.haversine_distance(x, y, self.prev_x, self.prev_y)
        # 이전 waypoint와의 거리가 0.3 이상이어야 기록
        if  distance > 0.5:
            data='{0}\t{1}\t{2}\n'.format(x,y,z)
            self.f.write(data)
            self.prev_x=x
            self.prev_y=y
            print("write : ", x,y,z)
    
    def haversine_distance(lat1, lon1, lat2, lon2):
        """
        Calculate the great circle distance in meters between two points 
        on the Earth's surface given their latitude and longitude in degrees.
        """
        # Convert latitude and longitude from degrees to radians
        lat1_rad = radians(lat1)
        lon1_rad = radians(lon1)
        lat2_rad = radians(lat2)
        lon2_rad = radians(lon2)
        
        # Earth radius in meters (assuming a spherical Earth)
        R = 6371000  # Approximate radius of the Earth in meters
        
        # Haversine formula
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        a = sin(dlat / 2)**2 + cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        distance = R * c
        
        return distance

    def status_callback(self,msg):
        self.is_status=True
        self.status_msg=msg

    
if __name__ == '__main__' :
    try:
        p_m=pathMaker("beginner_tutorials", "ego_path")
    except rospy.ROSInternalException:
        pass
            