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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32




class GoToPoint(Node):

  def __init__(self):
    
    super().__init__('go_to_point')

    self.global_path_publisher = self.create_subscription(Path, '/global_path', self.global_path, 10)

    self.publisherEngleGoaltoAuto = self.create_publisher(Float32, '/angle_auto_point', 10)

    self.subscription = self.create_subscription(
      Marker, 
      '/odomAuto', 
      self.listener_callback, 
      10)
    
    self.pathArr = np.empty((0,2))
    
    self.point_pub = self.create_subscription(PointStamped, '/clicked_point', self.listener_goal, 10)

    self.pubTwist = self.create_publisher(Twist, '/autocar/cmd_vel', 10)
    
    self.tfBuffer = tf2_ros.Buffer()
    self.tf = TransformListener(self.tfBuffer, self)
    #print("go")
    self.pubposemarker = self.create_publisher(Marker, '/goalPoint', 1)
    timer_period = 0.25
    self.timer = self.create_timer(timer_period, self.on_timer)
    self.x = None
    self.y = None
    self.engleAuto = 0.0
    self.tergetEngle = 0.0
    self.goalPosX = None
    self.goalPosY = None
    self.i = 0

  def global_path(self, data):
    self.pathArr = data.poses


  def listener_callback(self, data):
    self.x = data.pose.position.x
    self.y = data.pose.position.y
    euler = euler_from_quaternion(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)

    self.engleAuto = euler[2] 

  def listener_goal(self, data):
    self.goalPosX = data.point.x
    self.goalPosY = data.point.y

  def on_timer(self):
    if(len(self.pathArr) > 0):
      self.goalPosX = self.pathArr[self.i].pose.position.x
      self.goalPosY = self.pathArr[self.i].pose.position.y

    else:
      self.goalPosX = self.x
      self.goalPosY = self.x
    engleGoaltoAuto = -1
    VecGoalX = 1.0
    VecGoalY = 1.0
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
    if self.goalPosX == None:
      marker.pose.position.x = self.x
      marker.pose.position.y = self.y
    #elif self.goalPosX != None:
      #marker.pose.position.x = self.goalPosX
      #marker.pose.position.y = self.goalPosY
    marker.pose.position.z = 0.0
    self.pubposemarker.publish(marker)

    cEngle = 0.3
    linearSp = 0.0
    errEngle = math.pi/25
 
    VecAutoX = math.cos(self.engleAuto+ math.pi/2) #вектор авто
    VecAutoY = math.sin(self.engleAuto+ math.pi/2) 

    self.engleAuto += math.pi/2
    if self.engleAuto < 0:
      self.engleAuto = math.pi/2 - abs(self.engleAuto)+ 3*math.pi/2

    if self.goalPosX != None:
      VecGoalX = self.goalPosX - self.x
      VecGoalY = self.goalPosY - self.y
      engleGoaltoAuto = math.atan2(VecAutoX*VecGoalY - VecAutoY*VecGoalX, VecAutoX*VecGoalX + VecAutoY*VecGoalY)


      if abs(engleGoaltoAuto) < math.pi/2 :
        linearSp = 0.3
        if engleGoaltoAuto < 0:
          self.tergetEngle = convert(engleGoaltoAuto, 0, -math.pi/2, 0, -0.8)

        elif engleGoaltoAuto > 0:
          self.tergetEngle = convert(engleGoaltoAuto, 0, math.pi/2, 0, 0.8)

        else:
          self.tergetEngle = 0.0
        self.tergetEngle = 8*self.tergetEngle

      else:
        if(engleGoaltoAuto > 0):
            print(math.pi, engleGoaltoAuto)
            engleGoaltoAuto = (math.pi - engleGoaltoAuto)

        else:
            engleGoaltoAuto = (-math.pi - engleGoaltoAuto)


        linearSp = -0.3
        if engleGoaltoAuto < 0:
          self.tergetEngle = -convert(engleGoaltoAuto, 0, -math.pi/2, 0, -0.8)

        elif engleGoaltoAuto > 0:
          self.tergetEngle = -convert(engleGoaltoAuto, 0, math.pi/2, 0, 0.8)

        else:
          self.tergetEngle = 0.0

        self.tergetEngle = 5*self.tergetEngle

      
    print(self.tergetEngle)
    
    
    
    #print(engleGoaltoAuto*180/math.pi)
    msg = Float32()
    msg.data = engleGoaltoAuto
    self.publisherEngleGoaltoAuto.publish(msg)

    if IsPointInCircle(self.x, self.y, self.goalPosX, self.goalPosY, 0.3):
      self.i += 0

    
    twist = Twist()
    twist.linear.x = linearSp
    twist.linear.y = 0.0
    twist.linear.z = 0.0

    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = self.tergetEngle

    self.pubTwist.publish(twist)


def checkR(R,pointP, pointC):
      mode = ''
      boolCheck = False
      #print(math.degrees(pointP[2]))
      vec1X = math.cos(pointP[2] + math.pi/2)
      vec1Y = math.sin(pointP[2] + math.pi/2)

      #rotatedX = 17*math.sin(sP.a) + sP.x
      #rotatedY = 17*math.cos(sP.a) + sP.y

      vec2X = pointC[0]- pointP[0]
      vec2Y = pointC[1]- pointP[1]

      anglePC = angleV1V2(vec1X, vec1Y, vec2X, vec2Y)
      
      if anglePC <= 0:
         mode = 'R'
      else:
         mode = 'L'

      dist = distance(pointP[0], pointP[1], pointC[0], pointC[1])
      r = dist/(2*math.cos((math.pi/2)-abs(anglePC)))
      if(r >= R and abs(anglePC) < math.pi/2):
         boolCheck = True
      angleAcr = 2*anglePC
      angleNew = angleAcr

      L = abs(math.pi*r*math.degrees(angleAcr))/180
      info = [boolCheck, r, mode, angleAcr, angleNew, L]

      return info


def angleV1V2(vec1X, vec1Y, vec2X, vec2Y):
      c = math.atan2(vec1X*vec2Y - vec1Y*vec2X, vec1X*vec2X + vec1Y*vec2Y)
      return c
  
def distance(p1, p2):
      c = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
      return c
    
def convert(old, old_min, old_max, new_min, new_max):
  old_range = old_max - old_min  
  new_range = new_max - new_min  
  converted = (((old - old_min) * new_range) / old_range) + new_min
  return converted
    
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
  go_to_point = GoToPoint()
  
  # Spin the node so the callback function is called.
  rclpy.spin(go_to_point)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  go_to_point.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()