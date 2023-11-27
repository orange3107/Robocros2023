#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
import serial
import math
import time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Transform
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
import tf2_ros
from tf2_ros import TransformException 
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
import csv

with open('/home/ilya22/ros2_humble/src/robocross2023/paths/path2.csv', 'w', newline='') as file:
     writer = csv.writer(file)
     writer.writerow(["X", "Y", "A"])



class CreatePath(Node):

  def __init__(self):
    
    super().__init__('create_path')

    
    
    self.point_pub = self.create_subscription(PointStamped, '/clicked_point', self.listener_goal, 10)

    self.pubTwist = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
    
    self.tfBuffer = tf2_ros.Buffer()
    self.tf = TransformListener(self.tfBuffer, self)
    self.i = 0
    self.prev = None
    print("go")

  def listener_goal(self, data):
    print(self.i)
    self.i += 1
    self.goalPosX = data.point.x
    self.goalPosY = data.point.y

    if self.i > 1:

      a = angleV1V2(0, 1, data.point.x - self.prev[0], data.point.y - self.prev[1])
      with open('/home/ilya22/ros2_humble/src/robocross2023/paths/path2.csv', 'a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([self.prev[0], self.prev[1], a])       
        self.prev = [data.point.x, data.point.y]
        
    else:
      self.prev = [data.point.x, data.point.y]  

    
    


def angleV1V2(vec1X, vec1Y, vec2X, vec2Y):
      c = math.atan2(vec1X*vec2Y - vec1Y*vec2X, vec1X*vec2X + vec1Y*vec2Y)
      return c 

def IsPointInCircle(x, y, xc, yc, r):
    return ((x-xc)**2+(y-yc)**2) ** 0.5 <= r
  
      
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
  create_path = CreatePath()
  
  # Spin the node so the callback function is called.
  rclpy.spin(create_path)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  create_path.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()