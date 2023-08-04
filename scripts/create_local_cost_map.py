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
    self.scan_pub = self.create_publisher(LaserScan, '/scan3d', 1)
    self.subscription = self.create_subscription(
      PointCloud2,
      '/velodyne_points2', 
      self.listener_callback, 
      1)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
   

  
  def listener_callback(self, data):
    cp_array = sensor_msgs_py.point_cloud2.read_points(data, field_names=("x", "y", "z"))

    maxX = 0
    minX = 9999
    maxY = 0
    minY = 9999
    len_arr = len(cp_array)
    len_arr_filt = np.empty((0,2))
    scan_arr = np.empty((0,2))

    for i in range(len_arr-1, 0, -1):
             
        if(cp_array[i][2] > -1.6 and cp_array[i][1] > 0):

          #cp_array[i][0] *= 20.0
          #cp_array[i][1] *= 20.0
          dist = math.sqrt((cp_array[i][0])**2 + (cp_array[i][1])**2)

          cp_array[i][0] = int(cp_array[i][0])
          cp_array[i][1] = int(cp_array[i][1])
          
          cp_array[i][2] = 180/np.pi * math.atan2(cp_array[i][0], cp_array[i][1])

          scan_arr = np.append(scan_arr, [[dist, int(cp_array[i][2])]], axis = 0)

        else:
          cp_array = np.delete(cp_array, i, axis = 0)

    
    scan_arr = scan_arr[scan_arr[:, 1].argsort()]

    for i in range(len(scan_arr)-1, 0, -1):
       c1 = scan_arr[i][1]
       c2 = scan_arr[i-1][1]

       if c1 == c2: 
          if scan_arr[i][0] > scan_arr[i-1][0]:
             scan_arr = np.delete(scan_arr, i, axis = 0)

          else:
             scan_arr = np.delete(scan_arr, i-1, axis = 0)



    print("SORT")
    #print(scan_arr)
          
    current_time = self.get_clock().now().to_msg()

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'velodyne_link'
    scan.angle_min = -1.57
    scan.angle_max = 1.57
    scan.angle_increment = np.pi/180
    scan.time_increment = 0.1
    scan.range_min = 0.0
    scan.range_max = 100.0

    scan.ranges = []
    scan.intensities = []
    for i in range(-90, 90):
      b = False
      for j in range(len(scan_arr)):
        if scan_arr[j][1] == i :
          print(scan_arr[j][1])
          scan.ranges.append(scan_arr[j][0])
          b = True

      if b == False:
        print(-1)
        scan.ranges.append(0)
        
        scan.intensities.append(1)

    self.scan_pub.publish(scan)
    


    
    
     
    """
    for i in range(0, 180, +2):
       minDistance = 999999
       c = -1
       for j in range(len(cp_array)):
          
          if(checkPoint(200, cp_array[j][0], cp_array[j][1], i, i+1)):    
             dist = math.sqrt((cp_array[j][0])**2 + (cp_array[j][1])**2)
            
             if(dist < minDistance):              
                minDistance = dist
                c = j        

       if(c != -1):

        if(cp_array[c][0] > maxX):
            maxX = cp_array[c][0]

        if(cp_array[c][1] > maxY):
            maxY = cp_array[c][1]

        if(cp_array[c][0] < minX):
            minX = cp_array[c][0]

        if(cp_array[c][1] < minY):
            minY = cp_array[c][1]

        len_arr_filt = np.append(len_arr_filt, [[cp_array[c][0], cp_array[c][1]]], axis = 0)

    width = abs(int(maxX - minX))+1
    height = abs(int(maxY - minY))+1
    #print(width, height)
    #print(height, " ", width)
    image_map = np.zeros((width+100,height+100,3), np.uint8)

    image_map = cv2.rectangle(image_map, (0, 0), (height, width), (0, 0, 0), -1)

    
        
    for i in range(len(len_arr_filt)):
          
          old_range = maxX - minX # -10 - 90 
          new_range = width  # 0 100
          len_arr_filt[i][0] = (((len_arr_filt[i][0] - minX) * new_range) / old_range)
          centrX = (((0 - minX) * new_range) / old_range)

          old_range = maxY - minY # -10 - 90 
          new_range = height  # 0 100
          len_arr_filt[i][1] = (((len_arr_filt[i][1] - minY) * new_range) / old_range)
          centrY = (((0 - minY) * new_range) / old_range)
          

          image_map = cv2.rectangle(image_map, (int(len_arr_filt[i][0]), int(len_arr_filt[i][1])), (int(len_arr_filt[i][0])+1, int(len_arr_filt[i][1])+1), (100,100,100), -1)
          #image_map = cv2.rectangle(image_map, (int(centrX) - 40, int(centrY) + 70), (int(centrX)+20, int(centrY)), (100, 100, 0), -1)
    
    
    msg = OccupancyGrid()
    
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.info.map_load_time = self.get_clock().now().to_msg()
    msg.info.resolution = 0.05
    msg.header.frame_id = 'base_link'
    #msg.info.resolution = 0.1
    msg.info.height = image_map.shape[0]
    msg.info.width = image_map.shape[1]
    msg.info.origin.position.x = -centrX / 20.0
    msg.info.origin.position.y = -centrY / 20.0 
    msg.info.origin.position.z = 0.0

    msg.info.origin.orientation.x = 0.0
    msg.info.origin.orientation.y = 0.0
    msg.info.origin.orientation.z = 0.0
    msg.info.origin.orientation.w = 1.0

    image_map = cv2.cvtColor(image_map, cv2.COLOR_BGR2GRAY)
    img_data_int8array = [i for row in image_map.tolist() for i in row]
    for i in range(len(img_data_int8array)):
      if img_data_int8array[i] == 0:
        img_data_int8array[i] = -1
      
    msg.data = img_data_int8array
    self.cost_map.publish(msg)


    #image_map = cv2.cvtColor(image_map, cv2.COLOR_BGR2GRAY)
    image_map = cv2.flip(image_map,0)
    #cv2.imwrite('/home/ilya22/ros2_humble/cost_map.pgm', image_map)
    """

def checkPoint(radius, x, y, startAngle, endAngle):
 
    # calculate endAngle
    # calculate endAngle
    startAngle = startAngle*(np.pi/180)
    endAngle = endAngle*(np.pi/180)
    

    #print(startAngle, endAngle)
 
    # Calculate polar co-ordinates
    polarradius = math.sqrt(x * x + y * y)
    Angle = math.atan2(y, x)
    # Check whether polarradius is less
    # then radius of circle or not and
    # Angle is between startAngle and
    # endAngle or not
    if (Angle >= startAngle and Angle <= endAngle
                        and polarradius < radius):
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