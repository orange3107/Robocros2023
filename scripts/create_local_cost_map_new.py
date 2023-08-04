#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import serial
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import PointCloud2, PointField
import ros2_numpy
import open3d as o3d
import struct
import ctypes
import sensor_msgs_py.point_cloud2 
import math
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import LaserScan

class CreateCostMap(Node):

  def __init__(self):
    
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('local_map')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.cost_map = self.create_publisher(OccupancyGrid, '/local_cost_map', 1)
    self.sub_laserscan = self.create_subscription(
      LaserScan,
      '/scan', 
      self.sub_scan_callback, 
      1)
  
      
    # Used to convert between ROS and OpenCV images
   
  def sub_scan_callback(self, data):
     
     maxX = 0.0
     minX = 9999.0
     maxY = 0.0
     minY = 9999.0
     
     pointsXY = np.empty((0,2))
     print(data.ranges[0])

     for i in range(len(data.ranges)):
       if(data.ranges[i] < 100):
        x = int(data.ranges[i]*math.cos(data.angle_increment * i) * 20.0)
        y = int(data.ranges[i]*math.sin(data.angle_increment * i) * 20.0)
        pointsXY = np.append(pointsXY, [[x, y]], axis = 0)

        
        if(x> maxX):
            maxX = x

        if(y > maxY):
            maxY = y

        if(x < minX):
            minX = x

        if(y < minY):
            minY = y

     width = abs(int(maxX - minX))+1
     height = abs(int(maxY - minY))+1

     image_map = np.zeros((height,width,3), np.uint8)

     image_map = cv2.rectangle(image_map, (0, 0), (height, width), (0, 0, 0), -1)


     for i in range(len(pointsXY)):
          
          old_range = maxX - minX # -10 - 90 
          new_range = width  # 0 100
          pointsXY[i][0] = (((pointsXY[i][0] - minX) * new_range) / old_range)
          centrX = (((0 - minX) * new_range) / old_range)

          old_range = maxY - minY # -10 - 90 
          new_range = height  # 0 100
          pointsXY[i][1] = (((pointsXY[i][1] - minY) * new_range) / old_range)
          centrY = (((0 - minY) * new_range) / old_range)
          
          image_map = cv2.circle(image_map, (int(pointsXY[i][0]), int(pointsXY[i][1])), 10, (100,100,100), -1)
          #image_map = cv2.rectangle(image_map, (int(centrX) - 40, int(centrY) + 70), (int(centrX)+20, int(centrY)), (100, 100, 0), -1)
     #image_map = cv2.rotate(image_map, cv2.ROTATE_90_CLOCKWISE,0)
     msg = OccupancyGrid()
    
     msg.header.stamp = self.get_clock().now().to_msg()
     msg.info.map_load_time = self.get_clock().now().to_msg()
     msg.info.resolution = 0.05
     msg.header.frame_id = 'hokuyo_link'
     #msg.info.resolution = 0.1
     msg.info.height = image_map.shape[0]
     msg.info.width = image_map.shape[1]
     msg.info.origin.position.x = centrY/20
     msg.info.origin.position.y = centrX/20
     msg.info.origin.position.z = 0.0

     q = quaternion_from_euler(0, 0, -math.pi/2)
     msg.info.origin.orientation.x = q[0]
     msg.info.origin.orientation.y = q[1]
     msg.info.origin.orientation.z = q[2]
     msg.info.origin.orientation.w = q[3]

     image_map = cv2.cvtColor(image_map, cv2.COLOR_BGR2GRAY)
     img_data_int8array = [i for row in image_map.tolist() for i in row]
     for i in range(len(img_data_int8array)):
       if img_data_int8array[i] == 0:
        img_data_int8array[i] = -1
      
     msg.data = img_data_int8array
     self.cost_map.publish(msg)


    #image_map = cv2.cvtColor(image_map, cv2.COLOR_BGR2GRAY)

    #cv2.imwrite('/home/ilya22/ros2_humble/cost_map.pgm', image_map)  
     

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  local_map = CreateCostMap()
  
  # Spin the node so the callback function is called.
  rclpy.spin(local_map)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  local_map.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()