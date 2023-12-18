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

    width = 500 #abs(int(maxX - minX))+1
    height = 500 #abs(int(maxY - minY))+1

    self.image_map = np.zeros((height,width,3), np.uint8)
    self.image_map = cv2.rectangle(self.image_map, (0, 0), (height, width), (0, 0, 0), -1)
    

    cp_array = sensor_msgs_py.point_cloud2.read_points_numpy(data, field_names=("x", "y", "z"), skip_nans=True)

    print(cp_array)
    
    len_arr = len(cp_array)

    for i in range(len_arr-1, 0, -1):
        '''      
        if(cp_array[i][2] > -1.6 and cp_array[i][2] < 10 ):
          #print(cp_array[i], i)
          x = cp_array[i][0]*20.0 + width/2
          y = cp_array[i][1]*20.0 + height/2
          '''
        self.image_map = cv2.circle(self.image_map, (int(cp_array[i][0]), int(cp_array[i][1])), 5, (100,100,100), -1)
    



    print(len(cp_array))
     

    #print(width, height)
    #print(height, " ", width)
    centrX = width/2
    centrY = height/2

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
