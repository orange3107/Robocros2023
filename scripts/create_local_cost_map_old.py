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
from cv_bridge import CvBridge

class CreateCostMap(Node):

  def __init__(self):
    
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('local_map')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, '/map_image', 10)
    self.cost_map = self.create_publisher(OccupancyGrid, '/local_cost_map', 1)
    self.subscription = self.create_subscription(
      PointCloud2,
      '/velodyne_points', 
      self.listener_callback, 
      1)
    self.subscription # prevent unused variable warning
    width = 1
    height = 1
    self.image_map = np.zeros((width,height,3), np.uint8)
    self.br = CvBridge()
      
    # Used to convert between ROS and OpenCV images
   

  
  def listener_callback(self, data):
    cp_array = sensor_msgs_py.point_cloud2.read_points(data, field_names=("x", "y", "z"))

    maxX = 0
    minX = 9999
    maxY = 0
    minY = 9999
    len_arr = len(cp_array)
    width = 300 #abs(int(maxX - minX))+1
    height = 300 #abs(int(maxY - minY))+1
    
    scan_arr = np.empty((0,2))

    for i in range(len_arr-1, 0, -1):
             
        if(checkPoint(7, cp_array[i][0], cp_array[i][1]) and ((cp_array[i][0] > 1.5 or cp_array[i][1] > 3.5) or (cp_array[i][0] < -1.5 or cp_array[i][1] < -4)) and cp_array[i][2] > -1.6 and cp_array[i][2] < 10):

          cp_array[i][0] *= 20.0
          cp_array[i][1] *= 20.0

          cp_array[i][0] = int(cp_array[i][0]) + width/2
          cp_array[i][1] = int(cp_array[i][1]) + height/2
          
          if(cp_array[i][0] > maxX):
            maxX = cp_array[i][0]

          if(cp_array[i][1] > maxY):
            maxY = cp_array[i][1]

          if(cp_array[i][0] < minX):
            minX = cp_array[i][0]

          if(cp_array[i][1] < minY):
            minY = cp_array[i][1]


          
        else:
          cp_array = np.delete(cp_array, i, axis = 0)



    print(len(cp_array))
     

    #print(width, height)
    #print(height, " ", width)
    self.image_map = np.zeros((height,width,3), np.uint8)

    self.image_map = cv2.rectangle(self.image_map, (0, 0), (height, width), (0, 0, 0), -1)
        
    for i in range(len(cp_array)):
          
          old_range = maxX - minX # -10 - 90 
          new_range = width  # 0 100
          #cp_array[i][0] = (((cp_array[i][0] - minX) * new_range) / old_range)
          centrX = width/2#(((0 - minX) * new_range) / old_range)

          old_range = maxY - minY # -10 - 90 
          new_range = height  # 0 100
          #cp_array[i][1] = (((cp_array[i][1] - minY) * new_range) / old_range)
          centrY = height/2#(((0 - minY) * new_range) / old_range)

          #cp_array_list[int(cp_array[i][1])-1][int(cp_array[i][0])-1] = 100
          if(checkPoint(5, cp_array[i][0] - centrX, cp_array[i][1] - centrY) == False):
            self.image_map = cv2.circle(self.image_map, (int(cp_array[i][0]), int(cp_array[i][1])), 5, (100,100,100), -1)
          #image_map = cv2.rectangle(image_map, (int(centrX) - 40, int(centrY) + 70), (int(centrX)+20, int(centrY)), (100, 100, 0), -1)

    self.publisher_.publish(self.br.cv2_to_imgmsg(self.image_map))

    self.image_map = cv2.cvtColor(self.image_map, cv2.COLOR_BGR2GRAY)

    cp_array_list = np.array(self.image_map)
    cp_array_list = cp_array_list.ravel()
    cp_array_list = cp_array_list.tolist()
    msg = OccupancyGrid()
    
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.info.map_load_time = self.get_clock().now().to_msg()
    msg.info.resolution = 0.05
    msg.header.frame_id = 'base_link'
    #msg.info.resolution = 0.1
    msg.info.height = height
    msg.info.width = width
    msg.info.origin.position.x = -centrX/20
    msg.info.origin.position.y = -centrY/20
    msg.info.origin.position.z = 0.0

    msg.info.origin.orientation.x = 0.0
    msg.info.origin.orientation.y = 0.0
    msg.info.origin.orientation.z = 0.0
    msg.info.origin.orientation.w = 1.0

    #print(cp_array_list)
    msg.data = cp_array_list
    self.cost_map.publish(msg)


    #image_map = cv2.cvtColor(image_map, cv2.COLOR_BGR2GRAY)
    #self.image_map = cv2.flip(self.image_map,0)
    #cv2.imwrite('/home/ilya22/ros2_humble/cost_map.pgm', image_map)


def checkPoint(radius, x, y):
 
    polarradius = math.sqrt(x * x + y * y)
    # Check whether polarradius is less
    # then radius of circle or not and
    # Angle is between startAngle and
    # endAngle or not
    if (polarradius < radius):
        return True
    else:
        return False
 
# Driver code
  
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
