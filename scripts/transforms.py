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
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Transform

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster



class Transforms(Node):

  def __init__(self):
    
    super().__init__('transforms')

    self.subscription = self.create_subscription(
      PoseWithCovarianceStamped, 
      '/initialpose', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning

    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    
    
  def listener_callback(self, data):
      
      x = data.pose.pose.orientation.x
      y = data.pose.pose.orientation.y
      z = data.pose.pose.orientation.z
      w = data.pose.pose.orientation.w

      
      
      sinr_cosp = 2 * (w * x + y * z)
      cosr_cosp = 1 - 2 * (x * x + y * y)
      roll = np.arctan2(sinr_cosp, cosr_cosp)
 
      sinp = 2 * (w * y - z * x)
      pitch = np.arcsin(sinp)
 
      siny_cosp = 2 * (w * z + x * y)
      cosy_cosp = 1 - 2 * (y * y + z * z)
      yaw = np.arctan2(siny_cosp, cosy_cosp)-np.pi/2

      q = quaternion_from_euler(roll, pitch, yaw)

      msg = TransformStamped()

      msg.header.stamp = self.get_clock().now().to_msg()
      msg.header.frame_id = 'map'
      msg.child_frame_id = 'odom'

      msg.transform.translation.x = data.pose.pose.position.x
      msg.transform.translation.y = data.pose.pose.position.y
      msg.transform.translation.z = 0.0
        
      msg.transform.rotation.x = x
      msg.transform.rotation.y = y
      msg.transform.rotation.z = q[2]
      msg.transform.rotation.w = q[3]

      self.tf_static_broadcaster.sendTransform(msg)
      print(msg.transform.translation.x, " ", msg.transform.translation.y)

      


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
  transforms = Transforms()
  
  # Spin the node so the callback function is called.
  rclpy.spin(transforms)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  transforms.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()