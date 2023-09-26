#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import serial
import open3d as o3d

 
class PcdToMap(Node):

    def __init__(self):

        super().__init__('pcd_to_map')

    def convert(pcd):
      
      pcd_arr = np.asarray(pcd.points)
      maxX = 0
      minX = 9999
      maxY = 0
      minY = 9999
      minZ = 9999
 
      for i in range(len(pcd_arr)):
        if(pcd_arr[i][2] > 0.5): 
          pcd_arr[i][0] *= 20.0
          pcd_arr[i][1] *= 20.0

          if(pcd_arr[i][0] > maxX):
            maxX = pcd_arr[i][0]

          if(pcd_arr[i][1] > maxY):
            maxY = pcd_arr[i][1]

          if(pcd_arr[i][0] < minX):
            minX = pcd_arr[i][0]

          if(pcd_arr[i][1] < minY):
            minY = pcd_arr[i][1]

          if(pcd_arr[i][2] < minZ):
            minZ = pcd_arr[i][1]

      
      width = abs(int(maxX - minX))+1
      height = abs(int(maxY - minY))+1

      print(height)
      print(width)
      image_map = np.zeros((width,height,3), np.uint8)
      image_map = cv2.rectangle(image_map, (0, 0), (height, width), (255, 255, 255), -1)

      
      for i in range(len(pcd_arr)):

        old_range = maxX - minX # -10 - 90 
        new_range = width  # 0 100
        pcd_arr[i][0] = (((pcd_arr[i][0] - minX) * new_range) / old_range)

        old_range = maxY - minY # -10 - 90 
        new_range = height  # 0 100
        pcd_arr[i][1] = (((pcd_arr[i][1] - minY) * new_range) / old_range)

        

      """
        if(pcd_arr[i][2] > 0.2):

          #print(pcd_arr[i][0])
          #print(pcd_arr[i][1])
          image_map = cv2.rectangle(image_map, (int(pcd_arr[i][0]), int(pcd_arr[i][0] + 1)), (int(pcd_arr[i][1]), int(pcd_arr[i][1]) + 1), (255, 255, 255), -1)

        
      """
      for i in range(len(pcd_arr)):
        if(pcd_arr[i][2] > 0.9 and pcd_arr[i][2] < 2.5):
          print(pcd_arr[i][0])
          print(pcd_arr[i][1])

          image_map = cv2.circle(image_map, (int(pcd_arr[i][0]), int(pcd_arr[i][1])), 10, (0, 0, 0), -1)

      image_map = cv2.cvtColor(image_map, cv2.COLOR_BGR2GRAY)
      image_map = cv2.flip(image_map,1) 
      image_map = cv2.rotate(image_map, cv2.ROTATE_180)
      cv2.imwrite('/home/ilya22/ros2_humble/my_map.pgm', image_map)
       
       
    
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  in_pcd = o3d.io.read_point_cloud("/home/ilya22/map.pcd")
  pcd_to_map = PcdToMap.convert(in_pcd)
  # Spin the node so the callback function is called.
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  #pcd_to_map.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()