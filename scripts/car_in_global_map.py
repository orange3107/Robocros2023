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




class CarInGlobalMap(Node):

  def __init__(self):
    
    super().__init__('car_in_glaobal_map')
    
    self.tfBuffer = tf2_ros.Buffer()
    self.tf = TransformListener(self.tfBuffer, self)
    print("go")
    self.pubposemarker = self.create_publisher(Marker, '/poseAuto', 1)
    self.pubodomemarker = self.create_publisher(Marker, '/odomAuto', 1)
    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.on_timer)



  def on_timer(self):
    trans = None
     
    try:
      now = rclpy.time.Time()
      trans = self.tfBuffer.lookup_transform(
                  'map',
                  'velodyne',
                  now)
      marker = Marker()
      marker.header.frame_id = "/map"
      marker.type = marker.SPHERE
      marker.action = marker.ADD
      marker.scale.x = 0.2
      marker.scale.y = 0.2
      marker.scale.z = 0.2
      marker.color.a = 1.0
      marker.color.r = 1.0
      marker.color.g = 1.0
      marker.color.b = 0.0
      marker.pose.position.x = trans.transform.translation.x
      marker.pose.position.y = trans.transform.translation.y
      marker.pose.position.z = 0.0

      #print(trans.transform.translation.x, " ", trans.transform.translation.y)

      marker.pose.orientation.x = trans.transform.rotation.x
      marker.pose.orientation.y = trans.transform.rotation.y
      marker.pose.orientation.z = trans.transform.rotation.z
      marker.pose.orientation.w = trans.transform.rotation.w
      euler = euler_from_quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
      print(euler[2])
      self.pubposemarker.publish(marker)
    except TransformException as ex:
        print("err")

    try:
      now = rclpy.time.Time()
      trans = self.tfBuffer.lookup_transform(
                  'map',
                  'odometry',
                  now)
      marker = Marker()
      marker.header.frame_id = "/map"
      marker.type = marker.SPHERE
      marker.action = marker.ADD
      marker.scale.x = 0.2
      marker.scale.y = 0.2
      marker.scale.z = 0.2
      marker.color.a = 1.0
      marker.color.r = 1.0
      marker.color.g = 1.0
      marker.color.b = 0.0
      marker.pose.position.x = trans.transform.translation.x
      marker.pose.position.y = trans.transform.translation.y
      marker.pose.position.z = trans.transform.translation.z

      #print(trans.transform.translation.x, " ", trans.transform.translation.y)

      marker.pose.orientation.x = trans.transform.rotation.x
      marker.pose.orientation.y = trans.transform.rotation.y
      marker.pose.orientation.z = trans.transform.rotation.z
      marker.pose.orientation.w = trans.transform.rotation.w
      euler = euler_from_quaternion(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
      print("Odom", euler[2])
      self.pubodomemarker.publish(marker)
    except TransformException as ex:
        print("err")
    return
        
  

      
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
  car_in_global_map = CarInGlobalMap()
  
  # Spin the node so the callback function is called.
  rclpy.spin(car_in_global_map)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  car_in_global_map.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()