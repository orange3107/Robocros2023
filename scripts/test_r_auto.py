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
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from random import randint
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv

 
class TestRAuto(Node):

  def __init__(self):

    super().__init__('test')

    self.posX = None
    self.eulerAuto = None
    self.posX = None
    self.i = 0
    self.poses = np.empty((0,3))
    self.XY = np.empty((0,2))

    self.subscription = self.create_subscription(
      Marker, 
      '/poseAuto', 
      self.poseAuto_collback, 
      10)
    
    self.global_path_publisher = self.create_publisher(Path, '/test_path', 10)

    self.timer = self.create_timer(3.0, self.on_timer)

    with open('/home/ilya22/ros2_humble/src/robocross2023/paths/testRAuto.csv', 'w', newline='') as file:
               writer = csv.writer(file)
               writer.writerow(["X", "Y", "A"])
               
       
       
  def poseAuto_collback(self, data):
      
      euler = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
      self.eulerAuto = euler[2]
      #print(self.eulerAuto)

      self.posX = data.pose.position.x
      self.posY = data.pose.position.y

      self.XY = np.append(self.XY, [[self.posX, self.posY]], axis=0)

      msg = Path()
      msg.header.frame_id = "map"
      print(self.XY)
      if len(np.shape(self.XY)) > 1:
        for i in range(len(self.XY)):
          pose = PoseStamped()
          pose.pose.position.x = float(self.XY[i][0])
          pose.pose.position.y = float(self.XY[i][1])
          pose.pose.position.z = 0.0

          pose.pose.orientation.x = 0.0
          pose.pose.orientation.y = 0.0
          pose.pose.orientation.z = 0.0
          pose.pose.orientation.w = 0.0
          msg.poses.append(pose)

      self.global_path_publisher.publish(msg)
      


  def on_timer(self):
    if(self.i < 2):
      print("add")
      with open('/home/ilya22/ros2_humble/src/robocross2023/paths/testRAuto.csv', 'a', newline='') as file:
                  writer = csv.writer(file)
                  writer.writerow([self.posX, self.posY, self.eulerAuto])
      self.poses = np.append(self.poses, [[self.posX*20, self.posY*20, self.eulerAuto]],  axis=0)
      self.i += 1

    else:
        print(self.poses)
        info = self.checkR(0, self.poses[0], self.poses[1])
        print(info)
        self.i = 0
        self.poses = np.empty((0,3))


      
      

  def checkR(self, R,pointP, pointC):
      mode = ''
      boolCheck = False
      #print(math.degrees(pointP[2]))
      vec1X = math.cos(pointP[2] + math.pi/2)
      vec1Y = math.sin(pointP[2] + math.pi/2)

      #rotatedX = 17*math.sin(sP.a) + sP.x
      #rotatedY = 17*math.cos(sP.a) + sP.y

      vec2X = pointC[0]- pointP[0]
      vec2Y = pointC[1]- pointP[1]

      anglePC = self.angleV1V2(vec1X, vec1Y, vec2X, vec2Y)
      
      if anglePC <= 0:
         mode = 'R'
      else:
         mode = 'L'

      dist = self.distance(pointP[0], pointP[1], pointC[0], pointC[1])
      r = dist/(2*math.cos((math.pi/2)-abs(anglePC)))
      if(r >= R and abs(anglePC) < math.pi/2):
         boolCheck = True
      angleAcr = 2*anglePC
      angleNew = angleAcr

      L = abs(math.pi*r*math.degrees(angleAcr))/180
      info = [boolCheck, r, mode, angleAcr, angleNew, L]

      return info
  
  def angleV1V2(self, vec1X, vec1Y, vec2X, vec2Y):
      c = math.atan2(vec1X*vec2Y - vec1Y*vec2X, vec1X*vec2X + vec1Y*vec2Y)
      return c
  
  def distance(self, x1, y1, x2, y2):
      c = math.sqrt((x2-x1)**2 + (y2-y1)**2)
      return c
     



def euler_from_quaternion(x, y, z, w):
  euler = np.empty((3, ))
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)

  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)

  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)

  euler[0] = roll_x
  euler[1] = pitch_y
  euler[2] = yaw_z

  return euler     
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  test = TestRAuto()
  
  # Spin the node so the callback function is called.
  rclpy.spin(test)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  test.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()