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
from geometry_msgs.msg import Transform


class CreateGlobalMap(Node):

  def __init__(self):
    
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('global_map')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.image_map = cv2.imread('/home/ilya22/ros2_humble/src/robocross2023/maps/my_map.pgm')
    self.image_map = cv2.cvtColor(self.image_map, cv2.COLOR_BGR2GRAY)
    self.image_map = cv2.flip(self.image_map,0)
    self.image_map = cv2.rotate(self.image_map, cv2.ROTATE_180)
    self.global_map = self.create_publisher(OccupancyGrid, '/map', 1)
    # Used to convert between ROS and OpenCV images
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):

    msg = OccupancyGrid()
    
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.info.map_load_time = self.get_clock().now().to_msg()
    msg.info.resolution = 0.05
    msg.header.frame_id = 'map'
    #msg.info.resolution = 0.1
    msg.info.height = self.image_map.shape[0]
    msg.info.width = self.image_map.shape[1]
    msg.info.origin.position.x = float(self.image_map.shape[1])/20.0
    msg.info.origin.position.y = float(self.image_map.shape[0])/20.0
    msg.info.origin.position.z = 0.0
    msg.info.origin.position.z = 0.0

    msg.info.origin.orientation.x = 0.0
    msg.info.origin.orientation.y = 0.0
    msg.info.origin.orientation.z = 0.0
    msg.info.origin.orientation.w = 0.0

    img_data_int8array = [i for row in self.image_map.tolist() for i in row]

    for i in range(len(img_data_int8array)):
      
      if int(img_data_int8array[i]) == 255:
        img_data_int8array[i] = -1

      elif int(img_data_int8array[i]) == 0:
        print(int(img_data_int8array[i]))
        img_data_int8array[i] = 100

      elif int(img_data_int8array[i]) == 100:
        print(int(img_data_int8array[i]))
        img_data_int8array[i] = 101

    msg.data = img_data_int8array
    self.global_map.publish(msg)



def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  global_map = CreateGlobalMap()
  
  # Spin the node so the callback function is called.
  rclpy.spin(global_map)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  global_map.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()